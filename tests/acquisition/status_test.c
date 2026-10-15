/* The production status reader must work while a serial transaction owns lock. */
#include "core.h"
#include "ssf_mems_acquisition.h"
#include <assert.h>

unsigned long jiffies = 12345;

int main(void) {
  struct ssf_mems_xyzs_data sensor = {0};
  struct ssf_mems_acquisition_status status;
  struct ssf_mems_acquisition_state *acq = &sensor.acquisition;

  mutex_init(&acq->lock);
  mutex_init(&acq->status_lock);
  acq->online = acq->have_sample = true;
  acq->last_sample = 12000;
  acq->policy.interval_ms = 1234;

  /* Using the transaction mutex in get_status would deadlock this call. */
  mutex_lock(&acq->lock);
  ssf_mems_acquisition_get_status(&sensor, &status);
  assert(status.online && status.have_sample);
  assert(status.interval_ms == 1234 && status.sample_age_ms == 345);
  acq->have_sample = false;
  ssf_mems_acquisition_get_status(&sensor, &status);
  assert(!status.have_sample && status.sample_age_ms == 0);
  mutex_unlock(&acq->lock);
  assert(mutex_trylock(&acq->status_lock));
  mutex_unlock(&acq->status_lock);
  pthread_mutex_destroy(&acq->status_lock.native);
  pthread_mutex_destroy(&acq->lock.native);
  puts("PASS status reads while acquisition transaction lock is held");
  return 0;
}
