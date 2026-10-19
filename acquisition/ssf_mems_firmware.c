// SPDX-License-Identifier: GPL-2.0-only
#include "ssf_mems_acquisition.h"
#include "core.h"

#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/string.h>

/* 加载 bin 文件并升级传感器固件。
 * 返回：0 升级已确认，负数为文件、设备状态或传输错误。
 * 升级期间独占采集流程，末包结果不明时暂停自动重连。
 */
int ssf_mems_acquisition_update_firmware(struct ssf_mems_data *data,
                                         const char *name) {
  const struct firmware *firmware = NULL;
  struct ssf_mems_acquisition_state *acq;
  struct ssf_mems_ota_status status;
  char checked[SSF_MEMS_OTA_NAME_SIZE];
  int ret;

  if (data == NULL || data->serdev == NULL || name == NULL)
    return -EINVAL;
  if (data->sensor == NULL || !data->sensor->supports_ota)
    return -EOPNOTSUPP;
  ret = ssf_mems_ota_parse_name(name, strlen(name), checked);
  if (ret != 0)
    return ret;
  if (mutex_trylock(&data->ota.update_lock) == 0)
    return -EBUSY;
  ssf_mems_ota_set_stage(data, SSF_MEMS_OTA_LOADING, 0);
  if (READ_ONCE(data->ota.stopping) == true) {
    ret = -ENODEV;
    goto failed;
  }
  /* direct 不等待用户态 firmware fallback；缺失文件立即返回错误。 */
  ret = request_firmware_direct(&firmware, checked, &data->serdev->dev);
  if (ret != 0)
    goto failed;
  ret = ssf_mems_ota_validate_image(firmware->data, firmware->size);
  if (ret != 0)
    goto failed;
  ssf_mems_ota_set_stage(data, SSF_MEMS_OTA_WAITING, 0);
  acq = &data->acquisition;
  ret = mutex_lock_interruptible(&acq->lock);
  if (ret != 0)
    goto failed;
  if (READ_ONCE(acq->stopping) == true || READ_ONCE(data->ota.stopping) == true) {
    ret = -ENODEV;
    goto unlock_failed;
  }
  if (data->protocol.mode != SSF_MEMS_LINK_MODBUS) {
    ret = -EBUSY;
    goto unlock_failed;
  }
  /* 未上线时让后台先完成匹配；uncertain 后显式重试可执行只读预检。 */
  if (acq->online == false && acq->firmware_blocked == false) {
    ret = -ENODATA;
    goto unlock_failed;
  }
  mutex_lock(&acq->status_lock);
  acq->online = false;
  acq->have_sample = false;
  mutex_unlock(&acq->status_lock);
  ssf_mems_protocol_invalidate_features(data);
  ret = ssf_mems_ota_transfer(data, firmware->data, firmware->size);
  ssf_mems_ota_get_status(data, &status);
  /* 不确定的末包绝不由后台重连/切速来“修复”，以免隐式重启或重复写 Flash。 */
  if (status.stage == SSF_MEMS_OTA_UNCERTAIN)
    WRITE_ONCE(acq->firmware_blocked, true);
  else if (ret == 0)
    WRITE_ONCE(acq->firmware_blocked, false);
  WRITE_ONCE(acq->reconnect, true);
  wake_up_interruptible(&acq->waitq);
  mutex_unlock(&acq->lock);
  dev_info(&data->serdev->dev, "OTA %s: %zu/%zu bytes, version %d, error %d\n",
           ssf_mems_ota_stage_name(status.stage), status.acknowledged,
           status.total, status.version, ret);
  goto out;
unlock_failed:
  mutex_unlock(&acq->lock);
failed:
  if (ret == -ERESTARTSYS)
    ret = -EINTR;
  ssf_mems_ota_set_stage(data, SSF_MEMS_OTA_FAILED, ret);
out:
  if (firmware != NULL)
    release_firmware(firmware);
  mutex_unlock(&data->ota.update_lock);
  return ret;
}
