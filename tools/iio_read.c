// SPDX-License-Identifier: GPL-2.0-only
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <glob.h>
#include <limits.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define IIO_ROOT "/sys/bus/iio/devices"
#define DEVICE_NAME "ssf_mems_xyzs"
#define STANDARD_GRAVITY 9.80665 /* m/s^2 */

static volatile sig_atomic_t stopped; /* 信号处理置 1，主循环检查后安全退出。 */

/* 一个 sysfs 通道本轮的读数及启动时缓存的比例。 */
struct channel {
  double scale; /* 原始整数转物理单位的乘数，仅在用户态使用浮点。 */
  bool has_scale; /* 是否成功读取并解析比例。 */
  long long raw; /* 本轮读到的原始定点值。 */
  bool has_raw; /* 本轮原始值是否有效，避免显示上轮旧值。 */
};

/* 信号回调只置退出标志；无返回值，避免在信号上下文释放资源。 */
static void stop_reading(int signal_number) {
  (void)signal_number;
  stopped = 1;
}

/* 每次重开 sysfs；返回 0 成功，-EIO 空文件/未取得文本，
 * 其他 -errno 表示打开或读取失败（如 -ENOENT 文件不存在、-EACCES 无权限）。 */
static int read_text(const char *path, char *text, size_t size) {
  FILE *file = fopen(path, "r");
  int error;

  if (file == NULL)
    return -errno;
  errno = 0;
  if (fgets(text, (int)size, file) == NULL) {
    error = errno != 0 ? errno : EIO;
    fclose(file);
    return -error;
  }
  fclose(file);
  text[strcspn(text, "\r\n")] = '\0';
  return 0;
}

/* 拼接 sysfs 路径；返回 0 成功，-ENAMETOOLONG 格式化失败或缓冲区不足。 */
static int attribute_path(char *path, size_t size, const char *device,
                          const char *attribute) {
  int length = snprintf(path, size, "%s/%s", device, attribute);
  return length < 0 || (size_t)length >= size ? -ENAMETOOLONG : 0;
}

/* 按 IIO name 查设备；返回 0 找到，-ENODEV 无匹配/目录枚举失败，
 * -ENAMETOOLONG 找到的设备路径放不进输出缓冲区。 */
static int find_device(char *device, size_t size) {
  glob_t devices = {0};
  char path[PATH_MAX], name[128];
  int result = -ENODEV;
  size_t i;

  if (glob(IIO_ROOT "/iio:device*", 0, NULL, &devices) != 0) {
    globfree(&devices);
    return result;
  }
  for (i = 0; i < devices.gl_pathc; i++) {
    if (attribute_path(path, sizeof(path), devices.gl_pathv[i], "name") != 0 ||
        read_text(path, name, sizeof(name)) != 0 || strcmp(name, DEVICE_NAME) != 0)
      continue;
    if (strlen(devices.gl_pathv[i]) >= size) {
      result = -ENAMETOOLONG;
      break;
    }
    strcpy(device, devices.gl_pathv[i]);
    result = 0;
    break;
  }
  globfree(&devices);
  return result;
}

/* 解析十进制非负整数；返回 0 成功，-1 非数字开头、尾随字符或溢出。 */
static int parse_number(const char *text, unsigned long *value) {
  char *end;

  if (*text < '0' || *text > '9')
    return -1;
  errno = 0;
  *value = strtoul(text, &end, 10);
  return errno != 0 || *end != '\0' ? -1 : 0;
}

/* 打印本轮在线状态/轮询周期/样本年龄；无返回值，单项读取失败显示原因。 */
static void print_status(const char *device) {
  static const char *const attributes[] = {
      "sensor_online", "sensor_poll_interval_ms", "sensor_sample_age_ms"};
  char path[PATH_MAX], value[128];
  size_t i;
  int result;

  for (i = 0; i < sizeof(attributes) / sizeof(attributes[0]); i++) {
    result = attribute_path(path, sizeof(path), device, attributes[i]);
    if (result == 0)
      result = read_text(path, value, sizeof(value));
    if (result != 0)
      printf("  %s=unavailable (%s)", attributes[i], strerror(-result));
    else
      printf("  %s=%s", attributes[i], value);
  }
  printf("\n");
}

/* 打印命令行参数说明；无返回值。 */
static void usage(const char *program) {
  printf("Usage: %s [-d SYSFS_DEVICE] [-i INTERVAL_MS] [-n COUNT]\n"
         "  -d  IIO sysfs directory (default: discover " DEVICE_NAME ")\n"
         "  -i  Delay between reads in milliseconds (default: 100)\n"
         "  -n  Number of rounds (default: 0, run until Ctrl+C)\n"
         "  -h  Show this help\n", program);
}

/* 启动时打印一次传感器配置；无返回值，旧驱动缺失的属性直接跳过。 */
static void print_configuration(const char *device) {
  static const char *const attributes[] = {
      "sensor_firmware_version", "sensor_sampling_rate_index",
      "sensor_sampling_frequency", "sensor_sampling_length_index",
      "sensor_parameter_switch", "sensor_feature_enable"};
  char path[PATH_MAX], value[128];
  size_t i;
  int result;

  /* 这些属性会发 Modbus 请求，只在启动时读一次；兼容没有它们的旧模块。 */
  for (i = 0; i < sizeof(attributes) / sizeof(attributes[0]); i++) {
    result = attribute_path(path, sizeof(path), device, attributes[i]);
    if (result == 0)
      result = read_text(path, value, sizeof(value));
    if (result == 0)
      printf("  %s=%s\n", attributes[i], value);
    else if (result != -ENOENT)
      printf("  %s: unavailable (%s)\n", attributes[i], strerror(-result));
  }
}

/* 用本轮缓存值按 XYZ 汇总加速度，换算为 g；无返回值，不重复读取 sysfs。 */
static void print_acceleration(const glob_t *files,
                               const struct channel *channels) {
  static const char *const features[] = {
      "rms", "peak", "peak_to_peak", "high_freq_rms"};
  static const char *const labels[] = {
      "RMS", "Peak", "Peak-to-peak", "High-frequency RMS"};
  size_t feature, axis, i;

  printf("  Acceleration summary (g):\n");
  for (feature = 0; feature < sizeof(features) / sizeof(features[0]); feature++) {
    printf("    %s:", labels[feature]);
    for (axis = 0; axis < 3; axis++) {
      char expected[64];

      snprintf(expected, sizeof(expected), "in_accel_%c_%s_raw",
               (int)('x' + axis), features[feature]);
      for (i = 0; i < files->gl_pathc; i++) {
        const char *name = strrchr(files->gl_pathv[i], '/') + 1;

        if (strcmp(name, expected) == 0)
          break;
      }
      printf("  %c=", (int)('X' + axis));
      if (i < files->gl_pathc && channels[i].has_raw == true &&
          channels[i].has_scale == true)
        /* IIO 的 raw * scale 为 m/s^2，除以标准重力加速度得到 g。
         * 使用本轮已读取的数据，避免汇总时再次读取而跨越刷新边界。 */
        printf("%.2f g", (double)channels[i].raw * channels[i].scale /
                            STANDARD_GRAVITY);
      else
        printf("unavailable");
    }
    printf("\n");
  }
}

/* 解析参数并周期读 IIO；返回 EXIT_SUCCESS 正常结束/帮助，
 * EXIT_FAILURE 参数、设备枚举、内存、信号安装或休眠失败；单项读失败不退出。 */
int main(int argc, char **argv) {
  char device[PATH_MAX] = "", path[PATH_MAX], value[128];
  glob_t files = {0};
  struct channel *channels;
  struct sigaction action = {0};
  unsigned long interval_ms = 100, count = 0, round = 0;
  size_t i;
  int option, result;

  while ((option = getopt(argc, argv, "d:i:n:h")) != -1) {
    switch (option) {
    case 'd':
      if (strlen(optarg) >= sizeof(device)) {
        fprintf(stderr, "Device path is too long\n");
        return EXIT_FAILURE;
      }
      strcpy(device, optarg);
      break;
    case 'i':
      if (parse_number(optarg, &interval_ms) != 0 || interval_ms == 0 ||
          interval_ms > 3600000) {
        fprintf(stderr, "Interval must be between 1 and 3600000 ms\n");
        return EXIT_FAILURE;
      }
      break;
    case 'n':
      if (parse_number(optarg, &count) != 0) {
        fprintf(stderr, "Count must be a nonnegative integer\n");
        return EXIT_FAILURE;
      }
      break;
    case 'h':
      usage(argv[0]);
      return EXIT_SUCCESS;
    default:
      usage(argv[0]);
      return EXIT_FAILURE;
    }
  }
  if (optind != argc) {
    usage(argv[0]);
    return EXIT_FAILURE;
  }
  if (device[0] == '\0') {
    result = find_device(device, sizeof(device));
    if (result != 0) {
      fprintf(stderr, "Cannot find " DEVICE_NAME ": %s\n"
                      "Check that the driver is bound, or use -d SYSFS_DEVICE.\n",
              strerror(-result));
      return EXIT_FAILURE;
    }
  }
  result = attribute_path(path, sizeof(path), device, "in_*_raw");
  if (result != 0 || glob(path, 0, NULL, &files) != 0) {
    fprintf(stderr, "Cannot discover raw channels in %s\n", device);
    globfree(&files);
    return EXIT_FAILURE;
  }
  channels = calloc(files.gl_pathc, sizeof(*channels));
  if (channels == NULL) {
    perror("calloc");
    globfree(&files);
    return EXIT_FAILURE;
  }

  /* 本驱动每个带 scale 的通道都有独立的 *_scale 属性，启动时读取一次。 */
  for (i = 0; i < files.gl_pathc; i++) {
    char *end;
    size_t prefix_length = strlen(files.gl_pathv[i]) - strlen("raw");

    if (prefix_length + sizeof("scale") > sizeof(path))
      continue;
    memcpy(path, files.gl_pathv[i], prefix_length);
    strcpy(path + prefix_length, "scale");
    result = read_text(path, value, sizeof(value));
    if (result != 0) {
      if (result != -ENOENT)
        fprintf(stderr, "Cannot read %s: %s\n", path, strerror(-result));
      continue;
    }
    errno = 0;
    channels[i].scale = strtod(value, &end);
    channels[i].has_scale = errno == 0 && end != value && *end == '\0';
    if (channels[i].has_scale == false)
      fprintf(stderr, "Invalid scale in %s: %s\n", path, value);
  }

  action.sa_handler = stop_reading;
  sigemptyset(&action.sa_mask);
  if (sigaction(SIGINT, &action, NULL) != 0 ||
      sigaction(SIGTERM, &action, NULL) != 0) {
    perror("sigaction");
    free(channels);
    globfree(&files);
    return EXIT_FAILURE;
  }
  setvbuf(stdout, NULL, _IOLBF, 0);
  printf("Device: %s, channels: %zu, interval: %lu ms (Ctrl+C to stop)\n",
         device, files.gl_pathc, interval_ms);
  print_configuration(device);
  /* count=0 表示不限轮数，SIGINT/SIGTERM 均在完整清理后退出。 */
  while (stopped == 0 && (count == 0 || round < count)) {
    struct timespec delay = {.tv_sec = (time_t)(interval_ms / 1000),
                            .tv_nsec = (long)(interval_ms % 1000) * 1000000L};

    printf("\n--- round %lu ---\n", ++round);
    print_status(device);
    for (i = 0; i < files.gl_pathc && stopped == 0; i++) {
      const char *name = strrchr(files.gl_pathv[i], '/') + 1;
      char *end;
      long long raw;

      channels[i].has_raw = false;
      result = read_text(files.gl_pathv[i], value, sizeof(value));
      if (result != 0) {
        printf("  %s: unavailable (%s)\n", name, strerror(-result));
        continue;
      }
      errno = 0;
      raw = strtoll(value, &end, 10);
      if (errno != 0 || end == value || *end != '\0') {
        printf("  %s: invalid raw value '%s'\n", name, value);
        continue;
      }
      channels[i].raw = raw;
      channels[i].has_raw = true;
      printf("  %s: raw=%lld", name, raw);
      if (channels[i].has_scale == true)
        printf("  scale=%.9g  scaled=%.9g", channels[i].scale,
               (double)raw * channels[i].scale);
      if (strcmp(name, "in_count6_startup_flags_raw") == 0)
        printf("  running[X=%d Y=%d Z=%d]", (raw & 1) != 0,
               (raw & 2) != 0, (raw & 4) != 0);
      printf("\n");
    }
    if (stopped == 0)
      print_acceleration(&files, channels);
    if (count != 0 && round >= count)
      break;
    /* nanosleep 被信号打断时使用剩余时间，不从头重复整个间隔。 */
    while (stopped == 0 && nanosleep(&delay, &delay) < 0) {
      if (errno != EINTR) {
        perror("nanosleep");
        free(channels);
        globfree(&files);
        return EXIT_FAILURE;
      }
    }
  }
  free(channels);
  globfree(&files);
  return EXIT_SUCCESS;
}
