/*
 * 天腾航空大气检测项目
 *
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <termios.h>
#include <stdbool.h>
#include <errno.h>
#include <uORB/uORB.h>
//#include <uORB/topics/sensor_combined.h>
//#include <uORB/topics/vehicle_attitude.h>
/* 后添加的头文件 */
#include <uORB/topics/sensor_atmos.h>
//#include <ctime>


//#include <v2.0/sensor_atmos/mavlink_msg_sensor_atmos.h>


/* 定义主题 */
//ORB_DEFINE(rw_uart_sonar, struct rw_uart_sonar_data_s);

/* 以下程序摘自某博客 */
static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;

__EXPORT int tianteng_atmos_detect_main(int argc, char *argv[]);
int tianteng_atmos_detect_thread_main(int argc, char *argv[]);

static int uart_init(const char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);
static void usage(const char *reason);

int set_uart_baudrate(const int fd, unsigned int baud)
{
    int speed;

    switch (baud) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        default:
            warnx("ERR: baudrate: %d\n", baud);
            return -EINVAL;
    }

    struct termios uart_config;

    int termios_state;

    /* fill the struct for the new configuration */
    tcgetattr(fd, &uart_config);
    /* clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;
    /* no parity, one stop bit */
    uart_config.c_cflag &= ~(CSTOPB | PARENB);
    /* set baud rate */
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }

    if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
        warnx("ERR: %d (cfsetospeed)\n", termios_state);
        return false;
    }

    if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
        warnx("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }

    return true;
}


int uart_init(const char * uart_name)
{
    int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

    if (serial_fd < 0) {
        err(1, "failed to open port: %s", uart_name);
        return false;
    }
    return serial_fd;
}

static void usage(const char *reason)
{
    if (reason) {
        fprintf(stderr, "%s\n", reason);
    }

    fprintf(stderr, "usage: position_estimator_inav {start|stop|status} [param]\n\n");
    exit(1);
}

int tianteng_atmos_detect_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage("[YCM]missing command");
    }

    if (!strcmp(argv[1], "start")) {
        if (thread_running) {
            warnx("[YCM]already running\n");
            exit(0);
        }

        thread_should_exit = false;
        daemon_task = px4_task_spawn_cmd("tianteng_atmos_detect",
                         SCHED_DEFAULT,
                         SCHED_PRIORITY_MAX - 5,
                         1500,//2000,
						 tianteng_atmos_detect_thread_main,
                         (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            warnx("[YCM]running");

        } else {
            warnx("[YCM]stopped");
        }

        exit(0);
    }

    usage("unrecognized command");
    exit(1);
}

int tianteng_atmos_detect_thread_main(int argc, char *argv[])
{
    if (argc < 2) {
        errx(1, "[YCM]need a serial port name as argument");
        usage("eg:");
    }

    const char *uart_name = argv[1];

    warnx("[YCM]opening port %s", uart_name);
    char data = '0';
    char buffer[5] = "";
    /*
     * TELEM1 : /dev/ttyS1
     * TELEM2 : /dev/ttyS2
     * GPS    : /dev/ttyS3
     * NSH    : /dev/ttyS5
     * SERIAL4: /dev/ttyS6
     * N/A    : /dev/ttyS4
     * IO DEBUG (RX only):/dev/ttyS0
     */
    int uart_read = uart_init(uart_name);
    if(false == uart_read)return -1;
    if(false == set_uart_baudrate(uart_read,9600)){
        printf("[YCM]set_uart_baudrate is failed\n");
        return -1;
    }
    printf("[YCM]uart init is successful\n");

    thread_running = true;

    /*初始化数据结构体 */
    struct sensor_atmos_s sensor_atmos_data;
    memset(&sensor_atmos_data, 0, sizeof(sensor_atmos_data));
    /* 公告主题 */
    orb_advert_t sensor_atmos_pub = orb_advertise(ORB_ID(sensor_atmos), &sensor_atmos_data);
/////////////////////////////
FILE *fd = NULL;
char pname[64];
time_t timeSec = time(NULL);//1970.01.01
struct tm *timeinfo=localtime(&timeSec);
sprintf(pname, PX4_ROOTFSDIR"/fs/microsd/TIANTENG(%d_%d_%d).txt", timeinfo->tm_year+1900,timeinfo->tm_mon+1,timeinfo->tm_mday);
//sprintf(pname, PX4_ROOTFSDIR"/fs/microsd/TIANTENG.txt");

fd = fopen(pname,"a");//
fsync(fileno(fd));
fclose(fd);
/*
    FILE *f = fopen(PX4_ROOTFSDIR"/fs/microsd/inav.log", "a");
    if (f) {
        char *s = malloc(256);
        unsigned n = snprintf(s, 256,
                      "%llu %s\n\tdt=%.5f x_est=[%.5f %.5f] y_est=[%.5f %.5f] z_est=[%.5f %.5f] x_est_prev=[%.5f %.5f] y_est_prev=[%.5f %.5f] z_est_prev=[%.5f %.5f]\n",
                      (unsigned long long)hrt_absolute_time(), msg, (double)dt,
                      (double)x_est[0], (double)x_est[1], (double)y_est[0], (double)y_est[1], (double)z_est[0], (double)z_est[1],
                      (double)x_est_prev[0], (double)x_est_prev[1], (double)y_est_prev[0], (double)y_est_prev[1], (double)z_est_prev[0],
                      (double)z_est_prev[1]);
        fwrite(s, 1, n, f);
        n = snprintf(s, 256,
                 "\tacc=[%.5f %.5f %.5f] gps_pos_corr=[%.5f %.5f %.5f] gps_vel_corr=[%.5f %.5f %.5f] w_xy_gps_p=%.5f w_xy_gps_v=%.5f mocap_pos_corr=[%.5f %.5f %.5f] w_mocap_p=%.5f\n",
                 (double)acc[0], (double)acc[1], (double)acc[2],
                 (double)corr_gps[0][0], (double)corr_gps[1][0], (double)corr_gps[2][0], (double)corr_gps[0][1], (double)corr_gps[1][1],
                 (double)corr_gps[2][1],
                 (double)w_xy_gps_p, (double)w_xy_gps_v, (double)corr_mocap[0][0], (double)corr_mocap[1][0], (double)corr_mocap[2][0],
                 (double)w_mocap_p);
        fwrite(s, 1, n, f);
        n = snprintf(s, 256,
                 "\tvision_pos_corr=[%.5f %.5f %.5f] vision_vel_corr=[%.5f %.5f %.5f] w_xy_vision_p=%.5f w_z_vision_p=%.5f w_xy_vision_v=%.5f\n",
                 (double)corr_vision[0][0], (double)corr_vision[1][0], (double)corr_vision[2][0], (double)corr_vision[0][1],
                 (double)corr_vision[1][1], (double)corr_vision[2][1],
                 (double)w_xy_vision_p, (double)w_z_vision_p, (double)w_xy_vision_v);
        fwrite(s, 1, n, f);
        free(s);
    }

    fsync(fileno(f));
    fclose(f);
 */
/////////////////////////////
    while(!thread_should_exit){
        read(uart_read,&data,1);
        if(data == 'R'){
            for(int i = 0;i <4;i++){
                read(uart_read,&data,1);
                buffer[i] = data;
                data = '0';
            }
            //strncpy(sonardata.datastr,buffer,4);
            //sonardata.data = atoi(sonardata.datastr);
            sensor_atmos_data.CO = (buffer[0] & 0x0f) * 1000;
            sensor_atmos_data.CO += (buffer[1] & 0x0f) * 100;
            sensor_atmos_data.CO += (buffer[2] & 0x0f) * 10;
            sensor_atmos_data.CO += (buffer[3] & 0x0f);
            //printf("[YCM]sensor_atmos_data.CO=%d\n", sensor_atmos_data.CO);
            orb_publish(ORB_ID(sensor_atmos), sensor_atmos_pub, &sensor_atmos_data);
            write(uart_read,buffer,4);	//串口消息回传用于调试

            //存储数据
fd = fopen(pname,"a");//
///if (!fd){	//(fd != NULL) {
	timeSec = time(NULL);//1970.01.01
	timeinfo=localtime(&timeSec);
//fwrite("abcde", 5 , 5, fd);
	fprintf(fd,"CO：%04d\t", sensor_atmos_data.CO);
	fprintf(fd,"%d-%d-%d  %d:%d:%d  \n",timeinfo->tm_year+1900,timeinfo->tm_mon+1,timeinfo->tm_mday,timeinfo->tm_hour+8,timeinfo->tm_min,timeinfo->tm_sec);
//	fclose(fd);
//}
fsync(fileno(fd));
fclose(fd);
        }
//PX4_INFO("Hello Sky!");//printf("[YCM]qinbao\n");
//sleep(2);	//延时2秒
    }
//fclose(fd);

    warnx("[YCM]exiting");
    thread_running = false;
    close(uart_read);

    fflush(stdout);
    return 0;
}
