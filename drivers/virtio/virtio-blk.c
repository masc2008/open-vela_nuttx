/****************************************************************************
 * drivers/virtio/virtio-blk.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>
#include <errno.h>
#include <stdio.h>
#include <time.h>

#include <nuttx/cache.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/percpu.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <nuttx/virtio/virtio.h>
#include <nuttx/virtio/virtio-blk.h>
#include <nuttx/init.h>

#include "virtio-blk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_FS_LARGEFILE
#  define priblkcnt PRIu64
#else
#  define priblkcnt PRIu32
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct virtio_blk_priv_s
{
  FAR struct virtio_device     *vdev;           /* Virtio deivce */
  spinlock_t                    lock;           /* Lock */
  uint64_t                      nsectors;       /* Sectore numbers */
  uint32_t                      block_size;     /* Block size */
  char                          name[NAME_MAX]; /* Device name */
  virtio_blk_irq_cb_t           irq_sub_cb;     /* SUBSCRIBE_IRQ callback */
  FAR void                     *irq_sub_arg;    /* Cookie for irq_sub_cb */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* BLK block_operations functions and they helper function */

static ssize_t virtio_blk_rdwr(FAR struct virtio_blk_priv_s *priv,
                               FAR void *buffer, blkcnt_t startsector,
                               unsigned int nsectors, bool write);
static int     virtio_blk_open(FAR struct inode *inode);
static int     virtio_blk_close(FAR struct inode *inode);
static ssize_t virtio_blk_read(FAR struct inode *inode,
                               FAR unsigned char *buffer,
                               blkcnt_t startsector, unsigned int nsectors);
static ssize_t virtio_blk_write(FAR struct inode *inode,
                                FAR const unsigned char *buffer,
                                blkcnt_t startsector, unsigned int nsectors);
static int     virtio_blk_geometry(FAR struct inode *inode,
                                   FAR struct geometry *geometry);
static int     virtio_blk_ioctl(FAR struct inode *inode, int cmd,
                                unsigned long arg);
static int     virtio_blk_flush(FAR struct virtio_blk_priv_s *priv);

/* Other functions */

static int  virtio_blk_init(FAR struct virtio_blk_priv_s *priv,
                            FAR struct virtio_device *vdev);
static void virtio_blk_uninit(FAR struct virtio_blk_priv_s *priv);
static void virtio_blk_done(FAR struct virtqueue *vq);
static int  virtio_blk_probe(FAR struct virtio_device *vdev);
static void virtio_blk_remove(FAR struct virtio_device *vdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Device ID this driver probes against. Standard virtio spec value is
 * VIRTIO_ID_BLOCK (2); BES rptun backend advertises 251 for the block
 * class. Hard-coded here - bump if the rptun firmware changes.
 */

#define VIRTIO_BLK_MATCH_DEVICE_ID  251

static DEFINE_PER_CPU_BMP(struct virtio_driver, g_virtio_blk_driver) =
{
  VIRTIO_BLK_MATCH_DEVICE_ID,   /* device id */
  virtio_blk_probe,  /* probe */
  virtio_blk_remove, /* remove */
};
#define g_virtio_blk_driver this_cpu_var_bmp(g_virtio_blk_driver)

static const struct block_operations g_virtio_blk_bops =
{
  virtio_blk_open,     /* open     */
  virtio_blk_close,    /* close    */
  virtio_blk_read,     /* read     */
  virtio_blk_write,    /* write    */
  virtio_blk_geometry, /* geometry */
  virtio_blk_ioctl     /* ioctl    */
};

static DEFINE_PER_CPU_BMP(int, g_virtio_blk_idx);
#define g_virtio_blk_idx this_cpu_var_bmp(g_virtio_blk_idx)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_blk_wait_complete
 *
 * Description:
 *   Wait the virtio block request complete
 *
 ****************************************************************************/

static void virtio_blk_wait_complete(FAR struct virtqueue *vq,
                                     FAR sem_t *respsem)
{
  FAR struct virtio_blk_priv_s *priv = vq->vq_dev->priv;
  FAR sem_t *sem;

  if (up_interrupt_context() || OSINIT_IS_PANIC())
    {
      for (; ; )
        {
          sem = virtqueue_get_buffer_lock(vq, NULL, NULL, &priv->lock);
          if (sem == respsem)
            {
              break;
            }
          else if (sem != NULL)
            {
              nxsem_post(sem);
            }
        }
    }
  else
    {
      nxsem_wait_uninterruptible(respsem);
    }
}

/****************************************************************************
 * Name: virtio_blk_rdwr
 *
 * Description:
 *   Common function for read and write
 *
 ****************************************************************************/

/* resp shares its cache line with companion stack vars; LP writeback on
 * invalidate would clobber the peer's status. Force a 64B envelope.
 */

struct virtio_blk_resp_aligned_s
{
  struct virtio_blk_resp_s resp;
  uint8_t pad[64 - sizeof(struct virtio_blk_resp_s)];
} aligned_data(64);

static ssize_t virtio_blk_rdwr(FAR struct virtio_blk_priv_s *priv,
                               FAR void *buffer, blkcnt_t startsector,
                               unsigned int nsectors, bool write)
{
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  FAR struct virtqueue_buf vb[3];
  struct virtio_blk_resp_aligned_s respbuf
      aligned_data(64);
  struct virtio_blk_req_s req;
  irqstate_t flags;
  sem_t respsem;
  ssize_t ret;
  int readnum;

  nxsem_init(&respsem, 0, 0);

  /* Build the block request */

  req.type     = write ? VIRTIO_BLK_T_OUT : VIRTIO_BLK_T_IN;
  req.reserved = 0;
  req.sector   = startsector * priv->block_size >> VIRTIO_BLK_SECTOR_BITS;
  respbuf.resp.status = VIRTIO_BLK_S_IOERR;

  /* Fill the virtqueue buffer:
   * Buffer 0: the block out header;
   * Buffer 1: the read/write buffer;
   * Buffer 2: the block in header, return the status.
   */

  vb[0].buf = &req;
  vb[0].len = VIRTIO_BLK_REQ_HEADER_SIZE;
  vb[1].buf = buffer;
  vb[1].len = nsectors * priv->block_size;
  vb[2].buf = &respbuf.resp;
  vb[2].len = VIRTIO_BLK_RESP_HEADER_SIZE;
  readnum = write ? 2 : 1;

  /* Push req/resp from this CPU's D-cache out to physical memory before
   * the device peer (which observes memory through a different path -
   * e.g. ioremap on the host side) starts reading them. Without this,
   * the peer reads stale stack-coloration bytes (0xDEADBEEF) and parses
   * a bogus VIRTIO_BLK_T_* type.
   */

  up_clean_dcache((uintptr_t)&req, (uintptr_t)&req + sizeof(req));
  up_clean_dcache((uintptr_t)&respbuf,
                  (uintptr_t)&respbuf + sizeof(respbuf));

  if (up_interrupt_context())
    {
      virtqueue_disable_cb_lock(vq, &priv->lock);
    }

  flags = spin_lock_irqsave(&priv->lock);
  ret = virtqueue_add_buffer(vq, vb, readnum, 3 - readnum, &respsem);
  if (ret < 0)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      vrterr("virtqueue_add_buffer failed, ret=%zd\n", ret);
      goto err;
    }

  virtqueue_kick(vq);
  spin_unlock_irqrestore(&priv->lock, flags);

  /* Wait for the request completion */

  virtio_blk_wait_complete(vq, &respsem);

  /* Drop any cached copy of resp - the device peer wrote the new status
   * directly to physical memory, so we must invalidate before re-reading.
   */

  up_invalidate_dcache((uintptr_t)&respbuf,
                       (uintptr_t)&respbuf + sizeof(respbuf));

  if (respbuf.resp.status != VIRTIO_BLK_S_OK)
    {
      vrterr("%s Error: resp.status=0x%02x\n",
             write ? "Write" : "Read", respbuf.resp.status);
      ret = -EIO;
    }

err:
  if (up_interrupt_context())
    {
      virtqueue_enable_cb_lock(vq, &priv->lock);
    }

  return ret >= 0 ? nsectors : ret;
}

/****************************************************************************
 * Name: virtio_blk_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int virtio_blk_open(FAR struct inode *inode)
{
  DEBUGASSERT(inode->i_private);
  return OK;
}

/****************************************************************************
 * Name: virtio_blk_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int virtio_blk_close(FAR struct inode *inode)
{
  DEBUGASSERT(inode->i_private);
  return OK;
}

/****************************************************************************
 * Name: virtio_blk_read
 *
 * Description:
 *   Read the specified number of sectors from the read-ahead buffer or from
 *   the physical device.
 *
 ****************************************************************************/

static ssize_t virtio_blk_read(FAR struct inode *inode,
                               FAR unsigned char *buffer,
                               blkcnt_t startsector, unsigned int nsectors)
{
  FAR struct virtio_blk_priv_s *priv;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;
  return virtio_blk_rdwr(priv, buffer, startsector, nsectors, false);
}

/****************************************************************************
 * Name: virtio_blk_write
 *
 * Description:
 *   Write the specified number of sectors to the write buffer or to the
 *   physical device.
 *
 ****************************************************************************/

static ssize_t virtio_blk_write(FAR struct inode *inode,
                                FAR const unsigned char *buffer,
                                blkcnt_t startsector, unsigned int nsectors)
{
  FAR struct virtio_blk_priv_s *priv;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;
  if (virtio_has_feature(priv->vdev, VIRTIO_BLK_F_RO))
    {
      return -EPERM;
    }

  return virtio_blk_rdwr(priv, (FAR void *)buffer, startsector, nsectors,
                         true);
}

/****************************************************************************
 * Name: virtio_blk_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int virtio_blk_geometry(FAR struct inode *inode,
                               FAR struct geometry *geometry)
{
  FAR struct virtio_blk_priv_s *priv;
  int ret = -EINVAL;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  if (geometry)
    {
      geometry->geo_available    = true;
      geometry->geo_mediachanged = false;
      geometry->geo_writeenabled = true;
      geometry->geo_nsectors     = priv->nsectors;
      geometry->geo_sectorsize   = priv->block_size;
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_blk_ioctl
 ****************************************************************************/

static int virtio_blk_flush(FAR struct virtio_blk_priv_s *priv)
{
  FAR struct virtio_device *vdev = priv->vdev;
  FAR struct virtqueue *vq = vdev->vrings_info[0].vq;
  FAR struct virtqueue_buf vb[2];
  struct virtio_blk_resp_aligned_s respbuf
      aligned_data(64);
  struct virtio_blk_req_s req;
  irqstate_t flags;
  sem_t respsem;
  int ret;

  nxsem_init(&respsem, 0, 0);

  /* Build the block request */

  req.type     = VIRTIO_BLK_T_FLUSH;
  req.reserved = 0;
  req.sector   = 0;
  respbuf.resp.status = VIRTIO_BLK_S_IOERR;

  vb[0].buf = &req;
  vb[0].len = VIRTIO_BLK_REQ_HEADER_SIZE;
  vb[1].buf = &respbuf.resp;
  vb[1].len = VIRTIO_BLK_RESP_HEADER_SIZE;

  /* See virtio_blk_rdwr() for why these dcache ops are required. */

  up_clean_dcache((uintptr_t)&req, (uintptr_t)&req + sizeof(req));
  up_clean_dcache((uintptr_t)&respbuf,
                  (uintptr_t)&respbuf + sizeof(respbuf));

  flags = spin_lock_irqsave(&priv->lock);
  ret = virtqueue_add_buffer(vq, vb, 1, 1, &respsem);
  if (ret < 0)
    {
      spin_unlock_irqrestore(&priv->lock, flags);
      return ret;
    }

  virtqueue_kick(vq);
  spin_unlock_irqrestore(&priv->lock, flags);

  /* Wait for the request completion */

  virtio_blk_wait_complete(vq, &respsem);
  up_invalidate_dcache((uintptr_t)&respbuf,
                       (uintptr_t)&respbuf + sizeof(respbuf));

  if (respbuf.resp.status != VIRTIO_BLK_S_OK)
    {
      vrterr("Flush Error\n");
      ret = -EIO;
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_blk_ioctl
 ****************************************************************************/

static int virtio_blk_ioctl(FAR struct inode *inode, int cmd,
                            unsigned long arg)
{
  FAR struct virtio_blk_priv_s *priv;
  int ret = -ENOTTY;

  DEBUGASSERT(inode->i_private);
  priv = inode->i_private;

  switch (cmd)
    {
      case BIOC_FLUSH:
        if (virtio_has_feature(priv->vdev, VIRTIO_BLK_F_FLUSH))
          {
            ret = virtio_blk_flush(priv);
          }
        break;

      case VIRTIO_BLK_IOC_GET_DEVICE:
        if (arg == 0)
          {
            ret = -EINVAL;
          }
        else
          {
            *(FAR uintptr_t *)arg = (uintptr_t)priv->vdev;
            ret = OK;
          }
        break;

      case VIRTIO_BLK_IOC_SUBSCRIBE_IRQ:
        if (arg == 0)
          {
            ret = -EINVAL;
          }
        else
          {
            FAR struct virtio_blk_irq_sub_s *sub =
              (FAR struct virtio_blk_irq_sub_s *)arg;
            irqstate_t flags = spin_lock_irqsave(&priv->lock);
            priv->irq_sub_cb  = sub->cb;
            priv->irq_sub_arg = sub->arg;
            spin_unlock_irqrestore(&priv->lock, flags);
            ret = OK;
          }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: virtio_blk_done
 ****************************************************************************/

static void virtio_blk_done(FAR struct virtqueue *vq)
{
  FAR struct virtio_blk_priv_s *priv = vq->vq_dev->priv;
  FAR sem_t *respsem;
  virtio_blk_irq_cb_t cb;
  FAR void *cb_arg;
  irqstate_t flags;

  for (; ; )
    {
      respsem = virtqueue_get_buffer_lock(vq, NULL, NULL, &priv->lock);
      if (respsem == NULL)
        {
          break;
        }

      nxsem_post(respsem);
    }

  /* Snapshot (cb, arg) atomically under priv->lock so we never call
   * a new cb with a stale arg (or vice-versa) when a subscribe ioctl
   * races against this IRQ. The cb itself runs OUTSIDE the lock to
   * keep the critical section bounded; subscribers must therefore be
   * IRQ-safe (no blocking, no allocation, no nested take of priv->lock).
   * See <nuttx/virtio/virtio-blk.h> for the full subscriber contract.
   */

  flags  = spin_lock_irqsave(&priv->lock);
  cb     = priv->irq_sub_cb;
  cb_arg = priv->irq_sub_arg;
  spin_unlock_irqrestore(&priv->lock, flags);

  if (cb != NULL)
    {
      cb(cb_arg, vq->vq_dev);
    }
}

/****************************************************************************
 * Name: virtio_blk_init
 ****************************************************************************/

/* Read-modify-write wrapper around virtio_set_status(). The OpenAMP
 * remoteproc transport implements set_status() as an unconditional
 * 8-bit write; calling it directly would clobber any bits the remote
 * peer authored (BES uses bit 4/5 to signal AP backend readiness).
 */

static void virtio_blk_set_status_or(FAR struct virtio_device *vdev,
                                     uint8_t bit)
{
  uint8_t status = 0;

  if (virtio_get_status(vdev, &status) != 0)
    {
      status = 0;
    }

  virtio_set_status(vdev, status | bit);
}

static int virtio_blk_init(FAR struct virtio_blk_priv_s *priv,
                           FAR struct virtio_device *vdev)
{
  FAR const char *vqname[1];
  vq_callback callback[1];
  int ret;

  priv->vdev = vdev;
  vdev->priv = priv;
  spin_lock_init(&priv->lock);

  /* Initialize the virtio device */

  virtio_blk_set_status_or(vdev, VIRTIO_CONFIG_STATUS_DRIVER);
  virtio_negotiate_features(vdev, (1UL << VIRTIO_BLK_F_RO) |
                                  (1UL << VIRTIO_BLK_F_BLK_SIZE) |
                                  (1UL << VIRTIO_BLK_F_FLUSH), NULL);
  virtio_blk_set_status_or(vdev, VIRTIO_CONFIG_FEATURES_OK);

  vqname[0]   = "virtio_blk_vq";
  callback[0] = virtio_blk_done;
  ret = virtio_create_virtqueues(vdev, 0, 1, vqname, callback, NULL);
  if (ret < 0)
    {
      vrterr("virtio_device_create_virtqueue failed, ret=%d\n", ret);
      return ret;
    }

  /* The OpenAMP remoteproc transport leaves vq->shm_io NULL after
   * virtqueue_create(); without it virtqueue_virt_to_phys() in the IO
   * fast path dereferences NULL. Mirror what virtio-mmio_drv.c and
   * rpmsg_virtio.c do.
   */

  if (vdev->vrings_info[0].vq != NULL &&
      vdev->vrings_info[0].vq->shm_io == NULL)
    {
      virtqueue_set_shmem_io(vdev->vrings_info[0].vq,
                             vdev->vrings_info[0].io);
    }

  virtio_blk_set_status_or(vdev, VIRTIO_CONFIG_STATUS_DRIVER_OK);
  virtqueue_enable_cb(vdev->vrings_info[0].vq);
  return ret;
}

/****************************************************************************
 * Name: virtio_blk_uninit
 ****************************************************************************/

static void virtio_blk_uninit(FAR struct virtio_blk_priv_s *priv)
{
  FAR struct virtio_device *vdev = priv->vdev;

  virtio_reset_device(vdev);
  virtio_delete_virtqueues(vdev);
}

/****************************************************************************
 * Name: virtio_blk_probe
 ****************************************************************************/

static int virtio_blk_probe(FAR struct virtio_device *vdev)
{
  FAR struct virtio_blk_priv_s *priv;
  int ret;

  /* Alloc the virtio block driver private data */

  priv = kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      vrterr("No enough memory\n");
      return -ENOMEM;
    }

  /* Init the virtio block driver */

  ret = virtio_blk_init(priv, vdev);
  if (ret < 0)
    {
      vrterr("virtio_blk_init failed, ret=%d\n", ret);
      goto err_with_priv;
    }

  /* Read the block config and save the capacity to nsectors */

  virtio_read_config_member(priv->vdev, struct virtio_blk_config_s, capacity,
                            &priv->nsectors);
  vrtinfo("Virio blk capacity=%" PRIu64 " sectors\n", priv->nsectors);

  if (virtio_has_feature(vdev, VIRTIO_BLK_F_BLK_SIZE))
    {
      virtio_read_config_member(priv->vdev, struct virtio_blk_config_s,
                                blk_size, &priv->block_size);
      vrtinfo("Virio blk block_size=%" PRIu32 "\n", priv->block_size);
    }
  else
    {
      priv->block_size = VIRTIO_BLK_SECTOR_SIZE;
    }

  /* Register block driver */

  snprintf(priv->name, NAME_MAX, "/dev/virtblk%d", g_virtio_blk_idx);
  ret = register_blockdriver(priv->name, &g_virtio_blk_bops, 0660, priv);
  if (ret < 0)
    {
      vrterr("Register block driver failed, ret=%d\n", ret);
      goto err_with_init;
    }

  g_virtio_blk_idx++;
  return ret;

err_with_init:
  virtio_blk_uninit(priv);
err_with_priv:
  kmm_free(priv);
  return ret;
}

/****************************************************************************
 * Name: virtio_blk_remove
 ****************************************************************************/

static void virtio_blk_remove(FAR struct virtio_device *vdev)
{
  FAR struct virtio_blk_priv_s *priv = vdev->priv;

  unregister_driver(priv->name);
  virtio_blk_uninit(priv);
  kmm_free(priv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: virtio_register_blk_driver
 ****************************************************************************/

int virtio_register_blk_driver(void)
{
  return virtio_register_driver(&g_virtio_blk_driver);
}
