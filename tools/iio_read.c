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

static volatile sig_atomic_t stopped;

struct channel {
  double scale;
  bool has_scale;
  long long raw;
  bool has_raw;
};

static void stop_reading(int signal_number) {
  (void)signal_number;
  stopped = 1;
}

/* 每次重新打开 sysfs 属性，确保读取的是当前值。 */
static int read_text(const char *path, char *text, size_t size) {
  FILE *file = fopen(path, "r");
  int error;

  if (!file)
    return -errno;
  errno = 0;
  if (!fgets(text, (int)size, file)) {
    error = errno ? errno : EIO;
    fclose(file);
    return -error;
  }
  fclose(file);
  text[strcspn(text, "\r\n")] = '\0';
  return 0;
}

static int attribute_path(char *path, size_t size, const char *device,
                          const char *attribute) {
  int length = snprintf(path, size, "%s/%s", device, attribute);
  return length < 0 || (size_t)length >= size ? -ENAMETOOLONG : 0;
}

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
    if (attribute_path(path, sizeof(path), devices.gl_pathv[i], "name") ||
        read_text(path, name, sizeof(name)) || strcmp(name, DEVICE_NAME))
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

static int parse_number(const char *text, unsigned long *value) {
  char *end;

  if (*text < '0' || *text > '9')
    return -1;
  errno = 0;
  *value = strtoul(text, &end, 10);
  return errno || *end ? -1 : 0;
}

static void print_status(const char *device) {
  static const char *const attributes[] = {
      "sensor_online", "sensor_poll_interval_ms", "sensor_sample_age_ms"};
  char path[PATH_MAX], value[128];
  size_t i;
  int result;

  for (i = 0; i < sizeof(attributes) / sizeof(attributes[0]); i++) {
    result = attribute_path(path, sizeof(path), device, attributes[i]);
    if (!result)
      result = read_text(path, value, sizeof(value));
    if (result)
      printf("  %s=unavailable (%s)", attributes[i], strerror(-result));
    else
      printf("  %s=%s", attributes[i], value);
  }
  printf("\n");
}

static void usage(const char *program) {
  printf("Usage: %s [-d SYSFS_DEVICE] [-i INTERVAL_MS] [-n COUNT]\n"
         "  -d  IIO sysfs directory (default: discover " DEVICE_NAME ")\n"
         "  -i  Delay between reads in milliseconds (default: 100)\n"
         "  -n  Number of rounds (default: 0, run until Ctrl+C)\n"
         "  -h  Show this help\n", program);
}

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
    if (!result)
      result = read_text(path, value, sizeof(value));
    if (!result)
      printf("  %s=%s\n", attributes[i], value);
    else if (result != -ENOENT)
      printf("  %s: unavailable (%s)\n", attributes[i], strerror(-result));
  }
}

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

        if (!strcmp(name, expected))
          break;
      }
      printf("  %c=", (int)('X' + axis));
      if (i < files->gl_pathc && channels[i].has_raw && channels[i].has_scale)
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
      if (parse_number(optarg, &interval_ms) || !interval_ms ||
          interval_ms > 3600000) {
        fprintf(stderr, "Interval must be between 1 and 3600000 ms\n");
        return EXIT_FAILURE;
      }
      break;
    case 'n':
      if (parse_number(optarg, &count)) {
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
  if (!device[0]) {
    result = find_device(device, sizeof(device));
    if (result) {
      fprintf(stderr, "Cannot find " DEVICE_NAME ": %s\n"
                      "Check that the driver is bound, or use -d SYSFS_DEVICE.\n",
              strerror(-result));
      return EXIT_FAILURE;
    }
  }
  result = attribute_path(path, sizeof(path), device, "in_*_raw");
  if (result || glob(path, 0, NULL, &files) != 0) {
    fprintf(stderr, "Cannot discover raw channels in %s\n", device);
    globfree(&files);
    return EXIT_FAILURE;
  }
  channels = calloc(files.gl_pathc, sizeof(*channels));
  if (!channels) {
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
    if (result) {
      if (result != -ENOENT)
        fprintf(stderr, "Cannot read %s: %s\n", path, strerror(-result));
      continue;
    }
    errno = 0;
    channels[i].scale = strtod(value, &end);
    channels[i].has_scale = !errno && end != value && !*end;
    if (!channels[i].has_scale)
      fprintf(stderr, "Invalid scale in %s: %s\n", path, value);
  }

  action.sa_handler = stop_reading;
  sigemptyset(&action.sa_mask);
  if (sigaction(SIGINT, &action, NULL) || sigaction(SIGTERM, &action, NULL)) {
    perror("sigaction");
    free(channels);
    globfree(&files);
    return EXIT_FAILURE;
  }
  setvbuf(stdout, NULL, _IOLBF, 0);
  printf("Device: %s, channels: %zu, interval: %lu ms (Ctrl+C to stop)\n",
         device, files.gl_pathc, interval_ms);
  print_configuration(device);
  while (!stopped && (!count || round < count)) {
    struct timespec delay = {.tv_sec = (time_t)(interval_ms / 1000),
                            .tv_nsec = (long)(interval_ms % 1000) * 1000000L};

    printf("\n--- round %lu ---\n", ++round);
    print_status(device);
    for (i = 0; i < files.gl_pathc && !stopped; i++) {
      const char *name = strrchr(files.gl_pathv[i], '/') + 1;
      char *end;
      long long raw;

      channels[i].has_raw = false;
      result = read_text(files.gl_pathv[i], value, sizeof(value));
      if (result) {
        printf("  %s: unavailable (%s)\n", name, strerror(-result));
        continue;
      }
      errno = 0;
      raw = strtoll(value, &end, 10);
      if (errno || end == value || *end) {
        printf("  %s: invalid raw value '%s'\n", name, value);
        continue;
      }
      channels[i].raw = raw;
      channels[i].has_raw = true;
      printf("  %s: raw=%lld", name, raw);
      if (channels[i].has_scale)
        printf("  scale=%.9g  scaled=%.9g", channels[i].scale,
               (double)raw * channels[i].scale);
      if (!strcmp(name, "in_count6_startup_flags_raw"))
        printf("  running[X=%d Y=%d Z=%d]", !!(raw & 1),
               !!(raw & 2), !!(raw & 4));
      printf("\n");
    }
    if (!stopped)
      print_acceleration(&files, channels);
    if (count && round >= count)
      break;
    while (!stopped && nanosleep(&delay, &delay) < 0) {
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
