// SPDX-License-Identifier: GPL-2.0-only
/* Bounded wire capture for XYZ finite acquisition. No Flash writes. */
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/serdev.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/vmalloc.h>
#include <linux/workqueue.h>

struct wire {
 struct serdev_device *serial;
 struct work_struct work;
 spinlock_t lock;
 struct dentry *debug;
 struct debugfs_blob_wrapper blob;
};
static unsigned int baud = 128000;
module_param(baud, uint, 0444);
static unsigned int rate = 6;
module_param(rate, uint, 0444);
static unsigned int restore;
module_param(restore, uint, 0444);
static u16 crc16(const u8 *p, unsigned int n)
{
 u16 c=0xffff; unsigned int i,b;
 for(i=0;i<n;i++){ c^=p[i]; for(b=0;b<8;b++) c=c&1?(c>>1)^0xa001:c>>1; }
 return c;
}
static int receive(struct serdev_device *s,const u8 *buf,size_t n)
{
 struct wire *w=serdev_device_get_drvdata(s); unsigned long f; size_t take;
 spin_lock_irqsave(&w->lock,f);
 take=min_t(size_t,n,131072-w->blob.size);
 memcpy((u8 *)w->blob.data+w->blob.size,buf,take); w->blob.size+=take;
 spin_unlock_irqrestore(&w->lock,f); return n;
}
static const struct serdev_device_ops ops={.receive_buf=receive,.write_wakeup=serdev_device_write_wakeup};
static void send(struct wire *w,u8 fn,u16 addr,u16 value)
{
 u8 b[8]={1,fn,addr>>8,addr,value>>8,value}; u16 c=crc16(b,6);
 b[6]=c;b[7]=c>>8;
 dev_info(&w->serial->dev,"WIRE TX fn=%u addr=%u value=%u offset=%lu\n",fn,addr,value,w->blob.size);
 serdev_device_write(w->serial,b,8,HZ);serdev_device_wait_until_sent(w->serial,HZ);msleep(300);
}
static void work(struct work_struct *item)
{
 struct wire *w=container_of(item,struct wire,work);
 msleep(500);
 send(w,3,101,1);send(w,3,51,1);send(w,3,52,1);send(w,3,60,2);
 send(w,6,51,restore?9:rate);
 if(restore){send(w,6,52,2);send(w,3,52,1);send(w,3,51,1);}
 else {
  u8 b[13]={1,16,0,60,0,2,4,0,0,4,0}; u16 c=crc16(b,11);
  b[11]=c;b[12]=c>>8;
  serdev_device_write(w->serial,b,13,HZ);serdev_device_wait_until_sent(w->serial,HZ);msleep(300);
  send(w,3,60,2);
  send(w,3,59,1);
  msleep(12000);
  dev_info(&w->serial->dev,"WIRE STREAM END offset=%lu\n",w->blob.size);
  send(w,3,101,1);
 }
 dev_info(&w->serial->dev,"WIRE DONE bytes=%lu\n",w->blob.size);
}
static int probe(struct serdev_device *s)
{
 struct wire *w; char name[80];int ret;
 w=devm_kzalloc(&s->dev,sizeof(*w),GFP_KERNEL);if(!w)return -ENOMEM;
 w->blob.data=vzalloc(131072);if(!w->blob.data)return -ENOMEM;
 w->serial=s;spin_lock_init(&w->lock);INIT_WORK(&w->work,work);
 serdev_device_set_drvdata(s,w);serdev_device_set_client_ops(s,&ops);
 ret=serdev_device_open(s);if(ret)goto fail;
 serdev_device_set_flow_control(s,false);serdev_device_set_parity(s,SERDEV_PARITY_NONE);
 serdev_device_set_baudrate(s,baud);
 snprintf(name,sizeof(name),"mems-wire-%s",dev_name(&s->dev));w->debug=debugfs_create_dir(name,NULL);
 debugfs_create_blob("raw.bin",0400,w->debug,&w->blob);schedule_work(&w->work);return 0;
fail:vfree(w->blob.data);return ret;
}
static void remove(struct serdev_device *s)
{
 struct wire *w=serdev_device_get_drvdata(s);cancel_work_sync(&w->work);
 debugfs_remove_recursive(w->debug);serdev_device_close(s);vfree(w->blob.data);
}
static const struct of_device_id matches[]={{.compatible="sange-cbm,ssf-mems-xyz"},{}};
static struct serdev_device_driver wire_driver={.driver={.name="mems_xyz_wire",.of_match_table=matches},.probe=probe,.remove=remove};
module_serdev_device_driver(wire_driver);
MODULE_LICENSE("GPL");
