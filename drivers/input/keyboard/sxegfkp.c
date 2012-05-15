/*
 * sxegfkp.c - driver for 8x8 keypad controller in sxegfkp chips
 *
 * Copyright (C) 2012 Elettronica GF S.r.l.
 * Code reused of modules
 * drivers/gpio/sxegfkp.c  					Copyright (c) 2010, Code Aurora Forum
 * drivers/input/keyboard/twl4030_keypad.c 	Copyright (C) 2007 Texas Instruments, Inc.
 *
 * Code written by:
 * Andrea Collamati <andrea.collamati@elettronicagf.it>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#define DEBUG
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/i2c/sx150x.h>
#include <linux/slab.h>

/*
 * The SX1509EGF uses a SX1509 as keypad controller
 * configured as 8x8 keypad.
 * I/O 0-7 controls rows and are configured as open drain
 * I/O 8-15 read the columns and are configured as inputs with pull-ups
 */

#define SX1509EGF_ROWS	8	/* SX1509EGF hard limit */
#define SX1509EGF_COLS	8
#define SX1509EGF_KEYMAP_SIZE 64
#define SX1509EGF_ROWSHIFT	3

#define SX1509_REGOPULLUPA			0x07
#define SX1509_REGOPULLUPB			0x06
#define SX1509_REGOPENDRAINA		0x0B
#define SX1509_REGOPENDRAINB		0x0A
#define SX1509_REGDIRA				0x11
#define SX1509_REGDIRB				0x10
#define SX1509_REGCLOCK				0x1E
#define SX1509_REGKEYCONFIG1		0x25
#define SX1509_REGKEYCONFIG2		0x26
#define SX1509_REGKEYDATA1			0x27
#define SX1509_REGKEYDATA2			0x28
#define SX1509_REGDEBOUNCECONFIG	0x22
#define SX1509_REGDEBOUNCEENABLEA	0x24
#define SX1509_REGDEBOUNCEENABLEB	0x23
#define SX1509_REGKEYCONFIG1		0x25
#define SX1509_REGKEYCONFIG2		0x26


#define SX1509_REGRESET			0x7D


struct sxegfkp {
	unsigned short keymap[SX1509EGF_KEYMAP_SIZE];
	unsigned n_rows;
	unsigned n_cols;
	unsigned irq;
	struct i2c_client *client;
	struct input_dev *input;
};

static const struct i2c_device_id sxegfkp_id[] =
{
	{ "sxegfkp", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sxegfkp_id);

static s32 sxegfkp_i2c_write(struct i2c_client *client, u8 reg, u8 val) {
	s32 err = i2c_smbus_write_byte_data(client, reg, val);

	if (err < 0)
		dev_warn(&client->dev, "i2c write fail: can't write %02x to %02x: %d\n",
				val, reg, err);
	return err;
}

static s32 sxegfkp_i2c_read(struct i2c_client *client, u8 reg, u8 *val) {
	s32 err = i2c_smbus_read_byte_data(client, reg);

	if (err >= 0)
		*val = err;
	else
		dev_warn(&client->dev, "i2c read fail: can't read from %02x: %d\n",
				reg, err);
	return err;
}

static irqreturn_t sxegfkp_irq_thread_fn(int irq, void *dev_id) {
	struct sxegfkp *chip = (struct sxegfkp *) dev_id;
	unsigned int row;
	unsigned int col;
	unsigned int code;
	u8 regkeydata1;
	u8 regkeydata2;

	sxegfkp_i2c_read(chip->client, SX1509_REGKEYDATA1, &regkeydata1);
	sxegfkp_i2c_read(chip->client, SX1509_REGKEYDATA2, &regkeydata2);
	col = find_first_zero_bit(&regkeydata1,sizeof(regkeydata1));
	row = find_first_zero_bit(&regkeydata2,sizeof(regkeydata2));
	code = MATRIX_SCAN_CODE(row, col, SX1509EGF_ROWSHIFT);
//	printk(KERN_INFO "col = %d   row = %d  code = %d chip->keymap[code] = %d \n", col, row, code,chip->keymap[code]);
	input_report_key(chip->input, chip->keymap[code], 1);
	input_sync(chip->input);
	input_report_key(chip->input, chip->keymap[code], 0);
	input_sync(chip->input);

	return IRQ_HANDLED;
}

static int __devinit sxegfkp_program(struct sxegfkp *kp) {
	int err;
	u8 val;
	err = sxegfkp_i2c_write(kp->client, SX1509_REGRESET, 0x12);
	if (err < 0)
		return err;

	err = sxegfkp_i2c_write(kp->client, SX1509_REGRESET, 0x34);
	if (err < 0)
		return err;
	/* After reset all pin are inputs/no open drain/ no pull-ups/ no pull-down */

	/* Configure Rows as outputs open drains*/
	err = sxegfkp_i2c_write(kp->client, SX1509_REGDIRA, 0x00);
	if (err < 0)
		return err;
	err = sxegfkp_i2c_write(kp->client, SX1509_REGOPENDRAINA, 0xFF);
	if (err < 0)
		return err;

	/* Configure Columns with pull-ups */
	err = sxegfkp_i2c_write(kp->client, SX1509_REGOPULLUPB, 0xFF);
	if (err < 0)
		return err;

	/* Configure debouncing on columns */
	err = sxegfkp_i2c_write(kp->client, SX1509_REGDEBOUNCECONFIG, 0x05);
	if (err < 0)
		return err;

	err = sxegfkp_i2c_write(kp->client, SX1509_REGDEBOUNCEENABLEB,
			0xFF);
	if (err < 0)
		return err;

	/* Enable Internal 2MHz Oscillator */
	err = sxegfkp_i2c_write(kp->client, SX1509_REGCLOCK, 0x40);
	if (err < 0)
		return err;

	/* Scan Time 32ms x 2MHz/fOSC*/
	err = sxegfkp_i2c_write(kp->client, SX1509_REGKEYCONFIG1, 0x05);
	if (err < 0)
		return err;

	/*  Start Keypad engine Configure Keypad geometry 8x8 */
	val = ((kp->n_rows-1) << 3) |(kp->n_cols-1);
	err = sxegfkp_i2c_write(kp->client, SX1509_REGKEYCONFIG2, val);
	if (err < 0)
		return err;

	return 0;
}

static int __devinit sxegfkp_probe(struct i2c_client *client,
		const struct i2c_device_id *id) {

	struct sxegfkp_platform_data *pdata;
	struct sxegfkp *kp;
	static const u32 i2c_funcs = I2C_FUNC_SMBUS_BYTE_DATA
			| I2C_FUNC_SMBUS_WRITE_WORD_DATA;
	int error;
	struct input_dev *input;
	pdata = client->dev.platform_data;

	if (!pdata || !pdata->rows || !pdata->cols || pdata->rows > SX1509EGF_ROWS
			|| pdata->cols > SX1509EGF_COLS || !pdata->keymap_data || !pdata->irq) {
		dev_err(&client->dev, "Invalid platform_data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, i2c_funcs))
		return -ENOSYS;

	kp = kzalloc(sizeof(*kp), GFP_KERNEL);
	input = input_allocate_device();
	if (!kp || !input) {
		error = -ENOMEM;
		goto err1;
	}

	kp->client = client;
	kp->n_rows = pdata->rows;
	kp->n_cols = pdata->cols;
	kp->irq = pdata->irq;

	/* setup input device */
	__set_bit(EV_KEY, input->evbit);

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);


	input->name = "sxegfkp keypad";
	input->phys = "sxegfkp/input1";
	input->dev.parent = &client->dev;

	input->id.bustype = BUS_HOST;

	input->keycode = kp->keymap;
	input->keycodesize = sizeof(kp->keymap[0]);
	input->keycodemax = ARRAY_SIZE(kp->keymap);

	matrix_keypad_build_keymap(pdata->keymap_data, SX1509EGF_ROWSHIFT, input->keycode,
			input->keybit);

	error = input_register_device(input);
	if (error) {
		dev_err(&client->dev, "Unable to register sxegfkp device\n");
		goto err1;
	}
	kp->input = input;
	error = sxegfkp_program(kp);
	if (error)
		goto err2;

	error = request_threaded_irq(kp->irq, NULL, sxegfkp_irq_thread_fn,
			IRQF_SHARED | IRQF_TRIGGER_FALLING, "sxegfkp_irq", kp);
	if (error < 0) {
		goto err2;
	}

	i2c_set_clientdata(client, kp);

	return 0;

err2: input_unregister_device(input);
	input = NULL;
err1: input_free_device(input);
	kfree(kp);
	return error;
}
static int __devexit sxegfkp_remove(struct i2c_client *client) {
	struct sxegfkp *kp;

	kp = i2c_get_clientdata(client);

	free_irq(kp->irq, kp);
	input_unregister_device(kp->input);
	kfree(kp);

	return 0;
}

static struct i2c_driver sxegfkp_driver = { .driver = { .name = "sxegfkp",
		.owner = THIS_MODULE }, .probe = sxegfkp_probe, .remove =
		__devexit_p(sxegfkp_remove), .id_table = sxegfkp_id, };

static int __init sxegfkp_init(void) {
	return i2c_add_driver(&sxegfkp_driver);
}
subsys_initcall(sxegfkp_init);

static void __exit sxegfkp_exit(void) {
	return i2c_del_driver(&sxegfkp_driver);
}
module_exit(sxegfkp_exit);

MODULE_AUTHOR("Gregory Bean <gbean@codeaurora.org>");
MODULE_DESCRIPTION("Driver for Semtech SX1509EGF I2C GPIO Expanders");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:sxegfkp");

