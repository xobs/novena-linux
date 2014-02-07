/*
 * EEPROM Oops/Panic logger
 *
 * The EEPROM contains an innate page size that limits the amount of
 * data that can be written at once.  Thus, buckets must be a multiple
 * of the EEPROM page size.
 *
 * With a bucket size of 4096 and an EEPROM size of 64 kilobytes, you'll
 * get 16 buckets.  However, you might want to leave some space at the
 * beginning of EEPROM to use as storage for configuration data.
 *
 * A bucket contains a valid record if it has the EEPROM_KERNMSG_HEADER
 * magic value at the start.
 *
 * Copyright (C) 2014 Sean Cross <xobs@kosagi.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pstore.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/version.h>

#define EEPROM_BUCKET_SIG "-8<-"
#define MIN_MEM_SIZE 4096UL

static ulong record_size = MIN_MEM_SIZE;
module_param(record_size, ulong, 0400);
MODULE_PARM_DESC(record_size,
		"size of each dump done on oops/panic");

static ulong eeprom_offset = 4096;
module_param(eeprom_offset, ulong, 0400);
MODULE_PARM_DESC(eeprom_offset,
		"start of reserved EEPROM used to store oops/panic logs");

static ulong eeprom_size = 61440;
module_param(eeprom_size, ulong, 0400);
MODULE_PARM_DESC(eeprom_size,
		"size of reserved EEPROM used to store oops/panic logs");

static ulong eeprom_page_size = 128;
module_param(eeprom_page_size, ulong, 0400);
MODULE_PARM_DESC(eeprom_page_size,
		"size of individual EEPROM flash pages");

static ulong eeprom_settle_usec = 5000;
module_param(eeprom_settle_usec, ulong, 0400);
MODULE_PARM_DESC(eeprom_settle_usec,
		"microseconds to wait for the EEPROM to write data");

static int do_dump_oops = 1;
module_param(do_dump_oops, int, 0600);
MODULE_PARM_DESC(do_dump_oops,
		"set to 1 to dump oopses, 0 to only dump panics (default 1)");

struct eeprom_priv {
	unsigned long eeprom_size;
	unsigned long eeprom_page_size;
	unsigned long eeprom_offset;
	unsigned long eeprom_settle_usec;

	unsigned long current_record;

	size_t record_size;

	int do_dump_oops;

	struct pstore_info *pstore;

	struct i2c_client *i2c;
};

struct eeprom_bucket_header {
	u8 signature[4];
	u16 size;
	u8 compressed;
	u8 type;
	struct timespec time;
} __attribute__ ((__packed__));

struct eeprom_bucket {
	struct eeprom_bucket_header header;
	u64 id;
	u8 *data;
};

static int eeprom_pstore_open(struct pstore_info *psi)
{
	struct eeprom_priv *priv = psi->data;

	priv->current_record = 0;
	return 0;
}

static int eeprom_write_page(struct eeprom_priv *priv,
		u16 page, u8 length, const u8 *values)
{
	int ret;
	struct i2c_client *i2c = priv->i2c;
	u8 buffer[2 + length];

	buffer[0] = page >> 8;
	buffer[1] = page;
	if (length && values) {
		memcpy(&buffer[2], values, length);
		dev_dbg(&i2c->dev, "Writing %d bytes to address %d (0x %x %x)\n",
				sizeof(buffer), page, buffer[0], buffer[1]);
	}
	else {
		dev_dbg(&i2c->dev, "Setting read address to %d\n", page);
	}
	ret = i2c_master_send(i2c, buffer, sizeof(buffer));
	if (ret < 0) {
		dev_dbg(&i2c->dev, "Unable to write page: %d\n", ret);
		return ret;
	}


	/* Only sleep if data was written */
	if (length && values) {
		dev_dbg(&i2c->dev, "Page written successfully\n");
		usleep_range(priv->eeprom_settle_usec, priv->eeprom_settle_usec + 1000);
	}
	else
		dev_dbg(&i2c->dev, "Address set successfully\n");

	return 0;
}

static int eeprom_read_header(struct eeprom_priv *priv,
		struct eeprom_bucket_header *header, int offset)
{
	struct i2c_client *i2c = priv->i2c;
	int ret;
	/* Set the page to read */
	ret = eeprom_write_page(priv, offset, 0, NULL);
	if (ret < 0) {
		dev_err(&i2c->dev, "Unable to set EEPROM page: %d\n", ret);
		goto err_out;
	}

	dev_dbg(&priv->i2c->dev, "Reading %d bytes into header\n", sizeof(*header));
	ret = i2c_master_recv(i2c, (char *)header, sizeof(*header));
	if (ret < 0) {
		dev_err(&i2c->dev, "Unable to read EEPROM bucket: %d\n", ret);
		goto err_out;
	}

	return 0;

err_out:
	return ret;
}

static inline int id_to_offset(struct eeprom_priv *priv, int id)
{
	return priv->eeprom_offset + (id * priv->record_size);
}

static int eeprom_read_next_bucket(struct eeprom_priv *priv,
				struct eeprom_bucket *bucket)
{
	int ret;
	struct i2c_client *i2c = priv->i2c;
	ulong offset;

	/* No more records */
	if (id_to_offset(priv, priv->current_record) > priv->eeprom_size)
		return -ENOENT;

	bucket->id = priv->current_record;
	offset = id_to_offset(priv, priv->current_record);
	priv->current_record++;

	dev_dbg(&priv->i2c->dev, "Trying to read record %lld\n", bucket->id);

	/* * Read in the page header.
	 * NOTE: We assume the eeprom page size is greater than
	 * sizeof(eeprom_bucket).
	 */
	ret = eeprom_read_header(priv, &bucket->header, offset);
	if (ret < 0)
		goto err_out;

	/* If the signature doesn't match, then it's simply not a record */
	if (memcmp(bucket->header.signature,
			EEPROM_BUCKET_SIG,
			sizeof(bucket->header.signature))) {
		ret = -EAGAIN;
		dev_dbg(&priv->i2c->dev, "Record %lld invalid\n", bucket->id);
		goto err_out;
	}
	else
		dev_dbg(&priv->i2c->dev, "Record %lld has signature: %c%c%c%c\n",
				bucket->id,
				bucket->header.signature[0],
				bucket->header.signature[1],
				bucket->header.signature[2],
				bucket->header.signature[3]);

	bucket->data = kmalloc(bucket->header.size, GFP_KERNEL);
	if (!bucket->data) {
		ret = -ENOMEM;
		goto err_out;
	}

	ret = i2c_master_recv(i2c, bucket->data, bucket->header.size);
	if (ret < 0) {
		dev_err(&i2c->dev, "Unable to read EEPROM page: %d\n", ret);
		goto err_free;
	}

	dev_dbg(&priv->i2c->dev, "Valid record found\n");
	
	return 0;

err_free:
	kfree(bucket->data);
err_out:
	return ret;
}

static int eeprom_find_next_free_bucket(struct eeprom_priv *priv)
{
	int id;
	int ret;
	ulong offset;
	struct i2c_client *i2c = priv->i2c;
	struct eeprom_bucket_header header;

	/* Look for a free inode */
	for (id = 0; id_to_offset(priv, id) < priv->eeprom_size; id++) {

		offset = id_to_offset(priv, id);
		ret = eeprom_read_header(priv, &header, offset);
		if (ret < 0) {
			dev_err(&i2c->dev, "Unable to read EEPROM bucket %d: %d\n", id, ret);
			return ret;
		}

		/* If it has a valid signature, continue */
		dev_dbg(&i2c->dev, "Comparing %d bytes of signature [%c%c%c%c] against found value [%c%c%c%c]\n",
				sizeof(header.signature),
				EEPROM_BUCKET_SIG[0],
				EEPROM_BUCKET_SIG[1],
				EEPROM_BUCKET_SIG[2],
				EEPROM_BUCKET_SIG[3],
				header.signature[0],
				header.signature[1],
				header.signature[2],
				header.signature[3]);
		if (!memcmp(&header.signature, EEPROM_BUCKET_SIG,
				sizeof(header.signature))) {
			dev_dbg(&i2c->dev, "Signature found for bucket %d, moving on to next bucket\n", id);
			continue;
		}

		dev_dbg(&i2c->dev, "Bucket %d is free\n", id);
		return id;
	}
	return -1;
}

static int eeprom_write_bucket(struct eeprom_priv *priv,
				struct eeprom_bucket *bucket)
{
	u8 buffer[sizeof(bucket->header) + bucket->header.size];
	int eeprom_offset;
	int buffer_offset;
	int page_size = priv->eeprom_page_size;
	int id = bucket->id;
	int ret;
	struct i2c_client *i2c = priv->i2c;

	memcpy(buffer, &bucket->header, sizeof(bucket->header));
	memcpy(buffer + sizeof(bucket->header), bucket->data, bucket->header.size);

	eeprom_offset = id_to_offset(priv, id);
	buffer_offset = 0;

	dev_dbg(&i2c->dev, "Sanity-checking %d bytes of signature [%c%c%c%c] against signature to write [%c%c%c%c]\n",
			sizeof(bucket->header.signature),
			EEPROM_BUCKET_SIG[0],
			EEPROM_BUCKET_SIG[1],
			EEPROM_BUCKET_SIG[2],
			EEPROM_BUCKET_SIG[3],
			bucket->header.signature[0],
			bucket->header.signature[1],
			bucket->header.signature[2],
			bucket->header.signature[3]);
	while (buffer_offset < sizeof(buffer)) {
		ret = eeprom_write_page(priv, eeprom_offset + buffer_offset, page_size, buffer + buffer_offset);
		if (ret < 0) {
			dev_err(&i2c->dev, "Unable to write EEPROM page: %d\n", ret);
			goto err_out;
		}

		buffer_offset += page_size;
	}

	return 0;

err_out:
	return ret;
}

static int eeprom_write_next_bucket(struct eeprom_priv *priv,
				struct eeprom_bucket *bucket)
{
	int id;

	id = eeprom_find_next_free_bucket(priv);
	if (id < 0)
		return -ENOSPC;

	bucket->id = id;
	memcpy(bucket->header.signature, EEPROM_BUCKET_SIG,
		sizeof(bucket->header.signature));
	return eeprom_write_bucket(priv, bucket);
}

static ssize_t eeprom_pstore_read(u64 *id, enum pstore_type_id *type,
				   int *count, struct timespec *time,
				   char **buf, bool *compressed,
				   struct pstore_info *psi)
{
	struct eeprom_priv *priv = psi->data;
	struct eeprom_bucket bucket;
	int err;

	do {
		err = eeprom_read_next_bucket(priv, &bucket);
	} while (err == -EAGAIN);

	if (err < 0)
		return 0;

	*compressed = bucket.header.compressed;
	*time = bucket.header.time;
	*type = bucket.header.type;
	*id = bucket.id;
	*buf = bucket.data;

	return bucket.header.size;
}

static int notrace eeprom_pstore_write_buf(enum pstore_type_id type,
					    enum kmsg_dump_reason reason,
					    u64 *id, unsigned int part,
					    const char *buf,
					    bool compressed, size_t size,
					    struct pstore_info *psi)
{
	struct eeprom_priv *priv = psi->data;
	struct eeprom_bucket bucket;
	int ret;

	dev_dbg(&priv->i2c->dev, "Considering writing record %d/%d part %d, %d bytes...\n",
			type, reason, part, size);

	if ((type == PSTORE_TYPE_CONSOLE)
		|| (type == PSTORE_TYPE_FTRACE)
		|| (type == PSTORE_TYPE_DMESG && (reason == KMSG_DUMP_OOPS || reason == KMSG_DUMP_PANIC)))
		;
	else {
		dev_dbg(&priv->i2c->dev, "Record is a type we don't care about\n");
		return -EINVAL;
	}

//	if (part != 1) {
//		dev_dbg(&priv->i2c->dev, "Only care about part 1\n");
//		return -ENOSPC;
//	}

	/* Report zeroed timestamp if called before timekeeping has resumed. */
	if (__getnstimeofday(&bucket.header.time)) {
		bucket.header.time.tv_sec = 0;
		bucket.header.time.tv_nsec = 0;
	}
	
	bucket.header.type = type;
	bucket.header.compressed = compressed;
	bucket.header.size = size;
	bucket.data = (char *)buf;

	dev_dbg(&priv->i2c->dev, "Going to write it\n");
	ret = eeprom_write_next_bucket(priv, &bucket);

	dev_dbg(&priv->i2c->dev, "Result of writing it: %d\n", ret);
	return ret;
}

static int eeprom_pstore_erase(enum pstore_type_id type, u64 id, int count,
				struct timespec time, struct pstore_info *psi)
{
	struct eeprom_priv *priv = psi->data;
	struct i2c_client *i2c = priv->i2c;
	ulong offset;
	int ret;
	u8 buf[priv->eeprom_page_size];

	dev_dbg(&i2c->dev, "Erasing record %lld\n", id);
	memset(buf, 0, sizeof(buf));

	offset = id_to_offset(priv, id);
	ret = eeprom_write_page(priv, offset, sizeof(buf), buf);
	if (ret < 0) {
		dev_err(&i2c->dev, "Unable to erase page: %d\n", ret);
		return ret;
	}

	return 0;
}

static struct pstore_info eeprom_pstore_info = {
	.owner	= THIS_MODULE,
	.name	= "eepromoops",
	.open	= eeprom_pstore_open,
	.read	= eeprom_pstore_read,
	.write_buf	= eeprom_pstore_write_buf,
	.erase	= eeprom_pstore_erase,
};


static int eepromoops_i2c_probe(struct i2c_client *i2c,
                            const struct i2c_device_id *id)
{
        struct eeprom_priv *priv;
	int err = -EINVAL;

        priv = devm_kzalloc(&i2c->dev, sizeof(struct eeprom_priv), GFP_KERNEL);
        if (priv == NULL)
                return -ENOMEM;

	priv->eeprom_size = eeprom_size;
	priv->eeprom_offset = eeprom_offset;
	priv->eeprom_settle_usec = eeprom_settle_usec;
	priv->eeprom_page_size = eeprom_page_size;
	priv->record_size = record_size;
	priv->do_dump_oops = do_dump_oops;
	priv->i2c = i2c;
	priv->current_record = 0;

	priv->pstore = &eeprom_pstore_info;
	priv->pstore->data = priv;
	priv->pstore->bufsize = priv->record_size - sizeof(struct eeprom_bucket_header);
	priv->pstore->buf = kmalloc(priv->pstore->bufsize, GFP_KERNEL);
	spin_lock_init(&priv->pstore->buf_lock);
	if (!priv->pstore->buf) {
		pr_err("cannot allocate pstore buffer\n");
		err = -ENOMEM;
		goto fail_out;
	}

	err = pstore_register(priv->pstore);
	if (err) {
		pr_err("registering with pstore failed\n");
		goto fail_out;
	}

        i2c_set_clientdata(i2c, priv);

	return 0;

fail_out:
	return err;
}

static int eepromoops_i2c_remove(struct i2c_client *client)
{
#if 0
	/* TODO(kees): We cannot unload eepromoops since pstore doesn't support
	 * unregistering yet.
	 */
	struct eeprom_priv *priv = &oops_cxt;

	iounmap(priv->virt_addr);
	release_mem_region(priv->phys_addr, priv->size);
	priv->max_dump_cnt = 0;

	/* TODO(kees): When pstore supports unregistering, call it here. */
	kfree(priv->pstore->buf);
	priv->pstore->bufsize = 0;

	return 0;
#endif
	return -EBUSY;
}

static const struct i2c_device_id eepromoops_i2c_id[] = {
        { "eepromoops", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, eepromoops_i2c_id);

static const struct of_device_id eepromoops_of_match[] = {
	{ .compatible = "kosagi,eepromoops", },
	{ }
};
MODULE_DEVICE_TABLE(of, eepromoops_of_match);

static struct i2c_driver eepromoops_i2c_driver = {
	.driver = {
		.name	= "eepromoops",
		.owner	= THIS_MODULE,
		.of_match_table	= eepromoops_of_match,
	},
	.probe	= eepromoops_i2c_probe,
	.remove	= __exit_p(eepromoops_i2c_remove),
	.id_table	= eepromoops_i2c_id,
};

static int __init eepromoops_init(void)
{
	int ret;
	pr_debug("eeprom-oops: adding i2c driver\n");
	ret = i2c_add_driver(&eepromoops_i2c_driver);
	if (ret != 0) {
		pr_err("failed to register eepromoops I2C driver: %d\n", ret);
	}
	return 0;
}
module_init(eepromoops_init);

static void __exit eepromoops_exit(void)
{
	i2c_del_driver(&eepromoops_i2c_driver);
}
module_exit(eepromoops_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sean Cross <xobs@kosagi.com>");
MODULE_DESCRIPTION("EEPROM Oops/Panic logger/driver");
