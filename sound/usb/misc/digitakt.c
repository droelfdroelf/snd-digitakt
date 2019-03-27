/*
 * Digitakt Overbridge Driver
 * Copyright (c) Stefan Rehm <droelfdroelf@gmail.com>
 * most stuff copied & adapted from the ua101 driver
 *
 * This driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2.
 *
 * This driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/audio.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include "../usbaudio.h"
#include "../midi.h"

MODULE_DESCRIPTION("Elektron Digitakt driver");
MODULE_AUTHOR("Stefan Rehm <droelfdroelf@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_SUPPORTED_DEVICE("{{Elektron,Digitakt}}");

#define DT_SAMPLES_PER_URB	49	// 7 blocks of 7 samples per URB
#define DT_SAMPLES_PER_PACKET 7
#define DT_PLAYBACK_PACKET_LEN  88
#define DT_CAPTURE_PACKET_LEN  386

#define TRANSFER_OUT_DATA_SIZE 2112
#define TRANSFER_IN_DATA_SIZE 8832

/*
 * Should not be lower than the minimum scheduling delay of the host
 * controller.  Some Intel controllers need more than one frame; as long as
 * that driver doesn't tell us about this, use 1.5 frames just to be sure.
 */
#define MIN_QUEUE_LENGTH	12
/* Somewhat random. */
#define MAX_QUEUE_LENGTH	32
/*
 * This magic value optimizes memory usage efficiency for the UA-101's packet
 * sizes at all sample rates, taking into account the stupid cache pool sizes
 * that usb_alloc_coherent() uses.
 */
#define DEFAULT_QUEUE_LENGTH	21

//#define MAX_PACKET_SIZE	 DT_CAPTURE_PACKET_LEN * 7/* hardware specific */
#define MAX_MEMORY_BUFFERS	MAX_QUEUE_LENGTH

static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;
static unsigned int queue_length = 21;


module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "card index");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "enable card");
module_param(queue_length, uint, 0644);
MODULE_PARM_DESC(queue_length, "USB queue length in microframes, "
		 __stringify(MIN_QUEUE_LENGTH)"-"__stringify(MAX_QUEUE_LENGTH));

enum {
	INTF_PLAYBACK,
	INTF_CAPTURE,
	INTF_MIDI,

	INTF_COUNT = 5
};

/* bits in struct digitakt::states */
enum {
	USB_CAPTURE_RUNNING,
	USB_PLAYBACK_RUNNING,
	ALSA_CAPTURE_OPEN,
	ALSA_PLAYBACK_OPEN,
	ALSA_CAPTURE_RUNNING,
	ALSA_PLAYBACK_RUNNING,
	CAPTURE_URB_COMPLETED,
	PLAYBACK_URB_COMPLETED,
	DISCONNECTED,
};

struct digitakt {
	struct usb_device *dev;
	struct snd_card *card;
	struct usb_interface *intf[INTF_COUNT];
	int card_index;
	struct snd_pcm *pcm;
	struct list_head midi_list;
	u64 format_bit;
	unsigned int rate;
	unsigned int packets_per_second;
	spinlock_t lock;
	struct mutex mutex;
	unsigned long states;

	/* FIFO to synchronize playback rate to capture rate */
	unsigned int rate_feedback_start;
	unsigned int rate_feedback_count;
	u8 rate_feedback[MAX_QUEUE_LENGTH];

	struct list_head ready_playback_urbs;
	struct tasklet_struct playback_tasklet;
	wait_queue_head_t alsa_capture_wait;
	wait_queue_head_t rate_feedback_wait;
	wait_queue_head_t alsa_playback_wait;
	struct digitakt_stream {
		struct snd_pcm_substream *substream;
		unsigned int usb_pipe;
		unsigned int channels;
		unsigned int frame_bytes;
		unsigned int max_packet_bytes;
		unsigned int period_pos;
		unsigned int buffer_pos;
		unsigned int queue_length;
		struct digitakt_urb {
			struct urb urb;
//			struct usb_iso_packet_descriptor iso_frame_desc[1];
			struct list_head ready_list;
		} *urbs[MAX_QUEUE_LENGTH];
		struct {
			unsigned int size;
			void *addr;
			dma_addr_t dma;
		} buffers[MAX_MEMORY_BUFFERS];
	} capture, playback;
};

static DEFINE_MUTEX(devices_mutex);
static unsigned int devices_used;
static struct usb_driver digitakt_driver;

static void abort_alsa_playback(struct digitakt *dt);
static void abort_alsa_capture(struct digitakt *dt);

static const char *usb_error_string(int err)
{
	switch (err) {
	case -ENODEV:
		return "no device";
	case -ENOENT:
		return "endpoint not enabled";
	case -EPIPE:
		return "endpoint stalled";
	case -ENOSPC:
		return "not enough bandwidth";
	case -ESHUTDOWN:
		return "device disabled";
	case -EHOSTUNREACH:
		return "device suspended";
	case -EINVAL:
	case -EAGAIN:
	case -EFBIG:
	case -EMSGSIZE:
		return "internal error";
	default:
		return "unknown error";
	}
}

/*
 *  fill in dummy timestamp in playback data
 */
static void fill_in_meta_playback(u8* data, unsigned int len) {
	static u16 dummy_timestamp = 0;
	unsigned int offs = 0;
	snd_printd("fill_in_meta_playback");
	while (offs < len) {
		data[offs] = 0x07;
		data[offs + 1] = 0xFF;
		data[offs + 2] = (dummy_timestamp >> 8) & 0xFF;
		data[offs + 3] = (dummy_timestamp) & 0xFF;
		offs += 88;	// block length
		dummy_timestamp += 7;
	}
}

static void abort_usb_capture(struct digitakt *dt)
{
	snd_printd("abort_usb_capture");
	if (test_and_clear_bit(USB_CAPTURE_RUNNING, &dt->states)) {
		wake_up(&dt->alsa_capture_wait);
		wake_up(&dt->rate_feedback_wait);
	}
}

static void abort_usb_playback(struct digitakt *dt)
{
	snd_printd("abort_usb_playback");
	if (test_and_clear_bit(USB_PLAYBACK_RUNNING, &dt->states))
		wake_up(&dt->alsa_playback_wait);
}

static void playback_urb_complete(struct urb *usb_urb)
{
	struct digitakt_urb *urb = (struct digitakt_urb *) usb_urb;
	struct digitakt *dt = urb->urb.context;
	unsigned long flags;
	snd_printd("playback_urb_complete");
	if (unlikely(urb->urb.status == -ENOENT ||	/* unlinked */
		     urb->urb.status == -ENODEV ||	/* device removed */
		     urb->urb.status == -ECONNRESET ||	/* unlinked */
		     urb->urb.status == -ESHUTDOWN)) {	/* device disabled */
		abort_usb_playback(dt);
		abort_alsa_playback(dt);
		return;
	}

	if (test_bit(USB_PLAYBACK_RUNNING, &dt->states)) {
		/* append URB to FIFO */
		spin_lock_irqsave(&dt->lock, flags);
		list_add_tail(&urb->ready_list, &dt->ready_playback_urbs);
		if (dt->rate_feedback_count > 0)
			tasklet_schedule(&dt->playback_tasklet);
		dt->playback.substream->runtime->delay -= 1;
		//	DT_SAMPLES_PER_URB;// todo: fix for variable number of packets per urb
		spin_unlock_irqrestore(&dt->lock, flags);
	}
}

static void first_playback_urb_complete(struct urb *urb)
{
	struct digitakt *dt = urb->context;
	snd_printd("first_playback_urb_complete");
	urb->complete = playback_urb_complete;
	playback_urb_complete(urb);

	set_bit(PLAYBACK_URB_COMPLETED, &dt->states);
	wake_up(&dt->alsa_playback_wait);
}

/* copy data from the ALSA ring buffer into the URB buffer */
static bool copy_playback_data(struct digitakt_stream *stream, struct urb *urb,
			       unsigned int frames)
{
	struct snd_pcm_runtime *runtime;
	unsigned int frame_bytes, frames1;
	const u8 *source;
	snd_printd("copy_playback_data");
	runtime = stream->substream->runtime;
	frame_bytes = stream->frame_bytes;
	source = runtime->dma_area + stream->buffer_pos * frame_bytes;
	if (stream->buffer_pos + frames <= runtime->buffer_size) {
		memcpy(urb->transfer_buffer, source, frames * frame_bytes);
	} else {
		/* wrap around at end of ring buffer */
		frames1 = runtime->buffer_size - stream->buffer_pos;
		memcpy(urb->transfer_buffer, source, frames1 * frame_bytes);
		memcpy(urb->transfer_buffer + frames1 * frame_bytes,
		       runtime->dma_area, (frames - frames1) * frame_bytes);
	}

	stream->buffer_pos += frames;
	if (stream->buffer_pos >= runtime->buffer_size)
		stream->buffer_pos -= runtime->buffer_size;
	stream->period_pos += frames;
	if (stream->period_pos >= runtime->period_size) {
		stream->period_pos -= runtime->period_size;
		return true;
	}
	return false;
}

static inline void add_with_wraparound(struct digitakt *dt,
				       unsigned int *value, unsigned int add)
{
	*value += add;
	if (*value >= dt->playback.queue_length)
		*value -= dt->playback.queue_length;
}

static void playback_tasklet(unsigned long data)
{
	struct digitakt *dt = (void *) data;
	unsigned long flags;
	unsigned int frames;
	struct digitakt_urb *urb;
	bool do_period_elapsed = false;
	int err;

	if (unlikely(!test_bit(USB_PLAYBACK_RUNNING, &dt->states)))
		return;
	snd_printd("playback tasklet");
	/*
	 * Synchronizing the playback rate to the capture rate is done by using
	 * the same sequence of packet sizes for both streams.
	 * Submitting a playback URB therefore requires both a ready URB and
	 * the size of the corresponding capture packet, i.e., both playback
	 * and capture URBs must have been completed.  Since the USB core does
	 * not guarantee that playback and capture complete callbacks are
	 * called alternately, we use two FIFOs for packet sizes and read URBs;
	 * submitting playback URBs is possible as long as both FIFOs are
	 * nonempty.
	 */
	spin_lock_irqsave(&dt->lock, flags);
	while (dt->rate_feedback_count > 0 && !list_empty(&dt->ready_playback_urbs)) {
		/* take packet size out of FIFO */
		frames = dt->rate_feedback[dt->rate_feedback_start];
		add_with_wraparound(dt, &dt->rate_feedback_start, 7);
		dt->rate_feedback_count--;

		/* take URB out of FIFO */
		urb = list_first_entry(&dt->ready_playback_urbs,
				struct digitakt_urb, ready_list);
		list_del(&urb->ready_list);

		/* fill packet with data or silence */

		if (test_bit(ALSA_PLAYBACK_RUNNING, &dt->states))
			do_period_elapsed |= copy_playback_data(&dt->playback,
								&urb->urb,
								frames);
		else
			memset(urb->urb.transfer_buffer, 0,
					(frames / DT_SAMPLES_PER_PACKET) * DT_PLAYBACK_PACKET_LEN);

		// in any case, fill in header and dummy time stamp
		fill_in_meta_playback(urb->urb.transfer_buffer,
				(frames / DT_SAMPLES_PER_PACKET) * DT_PLAYBACK_PACKET_LEN);

		/* and off you go ... */
		snd_printd("usb_submit_urb");
		err = usb_submit_urb(&urb->urb, GFP_ATOMIC);
		if (unlikely(err < 0)) {
			spin_unlock_irqrestore(&dt->lock, flags);
			abort_usb_playback(dt);
			abort_alsa_playback(dt);
			dev_err(&dt->dev->dev, "USB request error %d: %s\n",
				err, usb_error_string(err));
			return;
		}
		dt->playback.substream->runtime->delay += frames;
	}
	spin_unlock_irqrestore(&dt->lock, flags);
	if (do_period_elapsed)
		snd_pcm_period_elapsed(dt->playback.substream);
}

/* copy data from the URB buffer into the ALSA ring buffer */
static bool copy_capture_data(struct digitakt_stream *stream, struct urb *urb,
			      unsigned int frames)
{
	struct snd_pcm_runtime *runtime;
	unsigned int frame_bytes, i;
	u8 *dest;
	struct digitakt *dt = urb->context;
	snd_printd("copy capture data");
	runtime = stream->substream->runtime;
	frame_bytes = stream->frame_bytes;

	if (frame_bytes != DT_SAMPLES_PER_URB * 4 * 12) {
		dev_dbg(&dt->dev->dev, "capture data: invalid size %i", frame_bytes);
		return false;
	}

	dest = runtime->dma_area + stream->buffer_pos * frame_bytes;
	if (stream->buffer_pos + frames <= runtime->buffer_size) {
		void* src = urb->transfer_buffer;
		for (i = 0; i < 8; i++) {
			src += 32; // skip block header
			memcpy(dest, src, 240);
			src += 240;
		}
	} else {
		/* wrap around at end of ring buffer */
//		frames1 = runtime->buffer_size - stream->buffer_pos;
//		memcpy(dest, urb->transfer_buffer, frames1 * frame_bytes);
//		memcpy(runtime->dma_area,
//		       urb->transfer_buffer + frames1 * frame_bytes,
//		       (frames - frames1) * frame_bytes);
		// this should not happen since we're using fixed block sizes!
		dev_dbg(&dt->dev->dev, "capture data: wrap around needed!");
	}

	stream->buffer_pos += frames;
	if (stream->buffer_pos >= runtime->buffer_size)
		stream->buffer_pos -= runtime->buffer_size;
	stream->period_pos += frames;
	if (stream->period_pos >= runtime->period_size) {
		stream->period_pos -= runtime->period_size;
		return true;
	}
	return false;
}

static void capture_urb_complete(struct urb *urb)
{
	struct digitakt *dt = urb->context;
	struct digitakt_stream *stream = &dt->capture;
	unsigned long flags;
	unsigned int frames, write_ptr;
	bool do_period_elapsed;
	int err;
	snd_printd("capture_urb_complete stat %i", urb->status);
	if (unlikely(urb->status == -ENOENT ||		/* unlinked */
		     urb->status == -ENODEV ||		/* device removed */
		     urb->status == -ECONNRESET ||	/* unlinked */
		     urb->status == -ESHUTDOWN))	/* device disabled */
		goto stream_stopped;

	if (urb->status >= 0)
		frames = 7 * 24;	// again hard coded ...
	else
		frames = 0;

	spin_lock_irqsave(&dt->lock, flags);

	if (frames > 0 && test_bit(ALSA_CAPTURE_RUNNING, &dt->states))
		do_period_elapsed = copy_capture_data(stream, urb, frames);
	else
		do_period_elapsed = false;

	if (test_bit(USB_CAPTURE_RUNNING, &dt->states)) {
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (unlikely(err < 0)) {
			spin_unlock_irqrestore(&dt->lock, flags);
			dev_err(&dt->dev->dev, "USB request error %d: %s\n",
				err, usb_error_string(err));
			goto stream_stopped;
		}

		/* append packet size to FIFO */
		write_ptr = dt->rate_feedback_start;
		add_with_wraparound(dt, &write_ptr, dt->rate_feedback_count);
		dt->rate_feedback[write_ptr] = frames;
		if (dt->rate_feedback_count < dt->playback.queue_length) {
			dt->rate_feedback_count++;
			if (dt->rate_feedback_count == dt->playback.queue_length)
				wake_up(&dt->rate_feedback_wait);
		} else {
			/*
			 * Ring buffer overflow; this happens when the playback
			 * stream is not running.  Throw away the oldest entry,
			 * so that the playback stream, when it starts, sees
			 * the most recent packet sizes.
			 */
					add_with_wraparound(dt, &dt->rate_feedback_start, 7);
		}
		if (test_bit(USB_PLAYBACK_RUNNING, &dt->states)
				&& !list_empty(&dt->ready_playback_urbs))
			tasklet_schedule(&dt->playback_tasklet);
	}

	spin_unlock_irqrestore(&dt->lock, flags);

	if (do_period_elapsed)
		snd_pcm_period_elapsed(stream->substream);

	return;

stream_stopped:
	abort_usb_playback(dt);
	abort_usb_capture(dt);
	abort_alsa_playback(dt);
	abort_alsa_capture(dt);
}

static void first_capture_urb_complete(struct urb *urb)
{
	struct digitakt *dt = urb->context;
	snd_printd("first_capture_urb_complete");
	urb->complete = capture_urb_complete;
	capture_urb_complete(urb);

	set_bit(CAPTURE_URB_COMPLETED, &dt->states);
	wake_up(&dt->alsa_capture_wait);
}

static int submit_stream_urbs(struct digitakt *dt,
		struct digitakt_stream *stream)
{
	unsigned int i;
	snd_printd("submit_stream_urbs");
	for (i = 0; i < stream->queue_length; ++i) {

		int err = usb_submit_urb(&stream->urbs[i]->urb, GFP_KERNEL);
		if (err < 0) {
			dev_err(&dt->dev->dev, "USB request error %d: %s\n",
				err, usb_error_string(err));
			return err;
		}
	}
	snd_printd("submit_stream_urbs OK");
	return 0;
}

static void kill_stream_urbs(struct digitakt_stream *stream)
{
	unsigned int i;
	snd_printd("kill_stream_urbs");
	for (i = 0; i < stream->queue_length; ++i)
		if (stream->urbs[i])
			usb_kill_urb(&stream->urbs[i]->urb);
}

static int enable_alt_setting(struct digitakt *dt, unsigned int intf_index,
		unsigned int altf)
{
	struct usb_host_interface *alts;
	dev_dbg(&dt->dev->dev, "setting alt setting for intf %i to %i", intf_index,
			altf);
	alts = dt->intf[intf_index]->cur_altsetting;
	if (alts->desc.bAlternateSetting != altf) {
		int err = usb_set_interface(dt->dev,
					    alts->desc.bInterfaceNumber, altf);
		if (err < 0) {
			dev_err(&dt->dev->dev,
				"cannot initialize interface; error %d: %s\n",
				err, usb_error_string(err));
			return err;
		}
	}
	return 0;
}

static void disable_alt_setting(struct digitakt *dt, unsigned int intf_index)
{
	struct usb_host_interface *alts;

	if (!dt->intf[intf_index])
		return;

	alts = dt->intf[intf_index]->cur_altsetting;
	if (alts->desc.bAlternateSetting != 0) {
		int err = usb_set_interface(dt->dev,
					    alts->desc.bInterfaceNumber, 0);
		if (err < 0 && !test_bit(DISCONNECTED, &dt->states))
			dev_warn(&dt->dev->dev,
				 "interface reset failed; error %d: %s\n",
				 err, usb_error_string(err));
	}
}

static void stop_usb_capture(struct digitakt *dt)
{
	snd_printd("stop usb capture");
	clear_bit(USB_CAPTURE_RUNNING, &dt->states);

	kill_stream_urbs(&dt->capture);

	//disable_alt_setting(dt, INTF_CAPTURE);
}

static int start_usb_capture(struct digitakt *dt)
{
	int err;
	snd_printd("start usb capture");
	if (test_bit(DISCONNECTED, &dt->states))
		return -ENODEV;

	if (test_bit(USB_CAPTURE_RUNNING, &dt->states))
		return 0;

	kill_stream_urbs(&dt->capture);

	clear_bit(CAPTURE_URB_COMPLETED, &dt->states);
	dt->capture.urbs[0]->urb.complete = first_capture_urb_complete;
	dt->rate_feedback_start = 0;
	dt->rate_feedback_count = 0;

	set_bit(USB_CAPTURE_RUNNING, &dt->states);
	err = submit_stream_urbs(dt, &dt->capture);
	if (err < 0)
		stop_usb_capture(dt);
	if (!err)
		snd_printd("start usb capture OK");
	return err;
}

static void stop_usb_playback(struct digitakt *dt)
{
	snd_printd("sstop usb playback");
	clear_bit(USB_PLAYBACK_RUNNING, &dt->states);

	kill_stream_urbs(&dt->playback);

	tasklet_kill(&dt->playback_tasklet);

	//disable_alt_setting(dt, INTF_PLAYBACK);
}

static int start_usb_playback(struct digitakt *dt)
{
	unsigned int i, frames;
	struct urb *urb;
	int err = 0;

	if (test_bit(DISCONNECTED, &dt->states))
		return -ENODEV;

	if (test_bit(USB_PLAYBACK_RUNNING, &dt->states))
		return 0;

	kill_stream_urbs(&dt->playback);
	tasklet_kill(&dt->playback_tasklet);

	clear_bit(PLAYBACK_URB_COMPLETED, &dt->states);
	dt->playback.urbs[0]->urb.complete =
		first_playback_urb_complete;
	spin_lock_irq(&dt->lock);
	INIT_LIST_HEAD(&dt->ready_playback_urbs);
	spin_unlock_irq(&dt->lock);

	/*
	 * We submit the initial URBs all at once, so we have to wait for the
	 * packet size FIFO to be full.
	 */
	snd_printd("playback wof");

	wait_event(dt->rate_feedback_wait,
			dt->rate_feedback_count >= dt->playback.queue_length
					|| !test_bit(USB_CAPTURE_RUNNING, &dt->states)
					|| test_bit(DISCONNECTED, &dt->states));
	if (test_bit(DISCONNECTED, &dt->states)) {
		stop_usb_playback(dt);
		return -ENODEV;
	}
	if (!test_bit(USB_CAPTURE_RUNNING, &dt->states)) {
		stop_usb_playback(dt);
		return -EIO;
	}

	for (i = 0; i < dt->playback.queue_length; ++i) {
		/* all initial URBs contain silence */
		spin_lock_irq(&dt->lock);
		frames = dt->rate_feedback[dt->rate_feedback_start];
				add_with_wraparound(dt, &dt->rate_feedback_start, 7);
		dt->rate_feedback_count--;
		spin_unlock_irq(&dt->lock);
		urb = &dt->playback.urbs[i]->urb;

		memset(urb->transfer_buffer, 0, 88 * 24);
		fill_in_meta_playback(urb->transfer_buffer, 88 * 24);
	}

	set_bit(USB_PLAYBACK_RUNNING, &dt->states);
	err = submit_stream_urbs(dt, &dt->playback);
	if (err < 0)
		stop_usb_playback(dt);
	if (!err)
		snd_printd("start usb playback OK");
	return err;
}

static void abort_alsa_capture(struct digitakt *dt)
{
	snd_printd("abort_alsa_capture");
	if (test_bit(ALSA_CAPTURE_RUNNING, &dt->states))
		snd_pcm_stop_xrun(dt->capture.substream);
}

static void abort_alsa_playback(struct digitakt *dt)
{
	snd_printd("abort_alsa_playback");
	if (test_bit(ALSA_PLAYBACK_RUNNING, &dt->states))
		snd_pcm_stop_xrun(dt->playback.substream);
}

static int set_stream_hw(struct digitakt *dt,
		struct snd_pcm_substream *substream,
			 unsigned int channels)
{
	int err;
	snd_printd("set_stream_hw");
	substream->runtime->hw.info =
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_FIFO_IN_FRAMES;
	substream->runtime->hw.formats = dt->format_bit;
	substream->runtime->hw.rates = snd_pcm_rate_to_rate_bit(dt->rate);
	substream->runtime->hw.rate_min = dt->rate;
	substream->runtime->hw.rate_max = dt->rate;
	substream->runtime->hw.channels_min = channels;
	substream->runtime->hw.channels_max = channels;
	substream->runtime->hw.periods_min = 2;
	substream->runtime->hw.periods_max = 16;

	substream->runtime->hw.buffer_bytes_max = 16 * DT_SAMPLES_PER_URB * 4
			* channels;
	// for now we support only a fixed buffer size
	substream->runtime->hw.period_bytes_min = 2 * DT_SAMPLES_PER_URB * 4
			* channels;
	substream->runtime->hw.period_bytes_max = 16 * DT_SAMPLES_PER_URB * 4
			* channels;
	err = snd_pcm_hw_constraint_minmax(substream->runtime,
					   SNDRV_PCM_HW_PARAM_PERIOD_TIME,
					   1500000 / dt->packets_per_second,
					   UINT_MAX);
	if (err < 0)
		return err;
	err = snd_pcm_hw_constraint_msbits(substream->runtime, 0, 32, 24);
	if (!err) {
		snd_printd("set_stream_hw OK");
	}
	return err;
}

static int capture_pcm_open(struct snd_pcm_substream *substream)
{
	struct digitakt *dt = substream->private_data;
	int err;
	snd_printd("capture_pcm_open");
	dt->capture.substream = substream;
	err = set_stream_hw(dt, substream, dt->capture.channels);
	if (err < 0)
		return err;
	substream->runtime->hw.fifo_size =
		DIV_ROUND_CLOSEST(dt->rate,
			dt->packets_per_second);
	substream->runtime->delay = substream->runtime->hw.fifo_size;

	mutex_lock(&dt->mutex);
	err = start_usb_capture(dt);
	if (err >= 0)
		set_bit(ALSA_CAPTURE_OPEN, &dt->states);
	mutex_unlock(&dt->mutex);
	if (!err) {
		snd_printd("capture_pcm_open OK");
	}
	return err;
}

static int playback_pcm_open(struct snd_pcm_substream *substream)
{
	struct digitakt *dt = substream->private_data;
	int err;
	snd_printd("playback_pcm_open");
	dt->playback.substream = substream;
	err = set_stream_hw(dt, substream, dt->playback.channels);
	if (err < 0)
		return err;
	substream->runtime->hw.fifo_size =
		DIV_ROUND_CLOSEST(
			dt->rate * dt->playback.queue_length, dt->packets_per_second);

	mutex_lock(&dt->mutex);
	err = start_usb_capture(dt);
	if (err < 0)
		goto error;
	err = start_usb_playback(dt);
	if (err < 0) {
		if (!test_bit(ALSA_CAPTURE_OPEN, &dt->states))
			stop_usb_capture(dt);
		goto error;
	}
	set_bit(ALSA_PLAYBACK_OPEN, &dt->states);
error:
	mutex_unlock(&dt->mutex);
	if (!err) {
		snd_printd("playback_pcm_open OK");
	}
	return err;
}

static int capture_pcm_close(struct snd_pcm_substream *substream)
{
	struct digitakt *dt = substream->private_data;

	mutex_lock(&dt->mutex);
	clear_bit(ALSA_CAPTURE_OPEN, &dt->states);
	if (!test_bit(ALSA_PLAYBACK_OPEN, &dt->states))
		stop_usb_capture(dt);
	mutex_unlock(&dt->mutex);
	snd_printd("capture_pcm_close");
	return 0;
}

static int playback_pcm_close(struct snd_pcm_substream *substream)
{
	struct digitakt *dt = substream->private_data;

	mutex_lock(&dt->mutex);
	stop_usb_playback(dt);
	clear_bit(ALSA_PLAYBACK_OPEN, &dt->states);
	if (!test_bit(ALSA_CAPTURE_OPEN, &dt->states))
		stop_usb_capture(dt);
	mutex_unlock(&dt->mutex);
	snd_printd("playback_pcm_close");
	return 0;
}

static int capture_pcm_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *hw_params)
{
	struct digitakt *dt = substream->private_data;
	int err;

	mutex_lock(&dt->mutex);
	err = start_usb_capture(dt);
	mutex_unlock(&dt->mutex);
	if (err < 0)
		return err;
	snd_printd("record_pcm_hw_params");
	return snd_pcm_lib_alloc_vmalloc_buffer(substream,
						params_buffer_bytes(hw_params));
}

static int playback_pcm_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *hw_params)
{
	struct digitakt *dt = substream->private_data;
	int err;

	mutex_lock(&dt->mutex);
	err = start_usb_capture(dt);
	if (err >= 0)
		err = start_usb_playback(dt);
	mutex_unlock(&dt->mutex);
	if (err < 0)
		return err;
	snd_printd("playback_pcm_hw_params");
	return snd_pcm_lib_alloc_vmalloc_buffer(substream,
						params_buffer_bytes(hw_params));
}

static int digitakt_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int capture_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct digitakt *dt = substream->private_data;
	int err;
	snd_printd("capture_pcm_prepare(..)");
	mutex_lock(&dt->mutex);
	err = start_usb_capture(dt);
	mutex_unlock(&dt->mutex);
	if (err < 0)
		return err;

	/*
	 * The EHCI driver schedules the first packet of an iso stream at 10 ms
	 * in the future, i.e., no data is actdtlly captured for that long.
	 * Take the wait here so that the stream is known to be actdtlly
	 * running when the start trigger has been called.
	 */
	wait_event(dt->alsa_capture_wait,
			test_bit(CAPTURE_URB_COMPLETED, &dt->states)
					|| !test_bit(USB_CAPTURE_RUNNING, &dt->states));
	if (test_bit(DISCONNECTED, &dt->states))
		return -ENODEV;
	if (!test_bit(USB_CAPTURE_RUNNING, &dt->states))
		return -EIO;
	snd_printd("capture_pcm_prepare(..) OK");
	dt->capture.period_pos = 0;
	dt->capture.buffer_pos = 0;
	return 0;
}

static int playback_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct digitakt *dt = substream->private_data;
	int err;
	snd_printd("playback_pcm_prepare(..)");
	mutex_lock(&dt->mutex);
	err = start_usb_capture(dt);
	if (err >= 0)
		err = start_usb_playback(dt);
	mutex_unlock(&dt->mutex);
	if (err < 0)
		return err;

	/* see the comment in capture_pcm_prepare() */
	wait_event(dt->alsa_playback_wait,
			test_bit(PLAYBACK_URB_COMPLETED, &dt->states)
					|| !test_bit(USB_PLAYBACK_RUNNING, &dt->states));
	if (test_bit(DISCONNECTED, &dt->states))
		return -ENODEV;
	if (!test_bit(USB_PLAYBACK_RUNNING, &dt->states))
		return -EIO;

	substream->runtime->delay = 0;
	dt->playback.period_pos = 0;
	dt->playback.buffer_pos = 0;
	snd_printd("playback_pcm_prepare(..) OK");
	return 0;
}

static int capture_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct digitakt *dt = substream->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		snd_printd("a trig STOP!");
		if (!test_bit(USB_CAPTURE_RUNNING, &dt->states))
			return -EIO;
		set_bit(ALSA_CAPTURE_RUNNING, &dt->states);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		snd_printd("a trig STOP!");
		clear_bit(ALSA_CAPTURE_RUNNING, &dt->states);
		return 0;
	default:
		return -EINVAL;
	}
}

static int playback_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct digitakt *dt = substream->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		snd_printd("p trig START!");
		if (!test_bit(USB_PLAYBACK_RUNNING, &dt->states))
			return -EIO;
		set_bit(ALSA_PLAYBACK_RUNNING, &dt->states);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		snd_printd("p trig STOP!");
		clear_bit(ALSA_PLAYBACK_RUNNING, &dt->states);
		return 0;
	default:
		return -EINVAL;
	}
}

static inline snd_pcm_uframes_t digitakt_pcm_pointer(struct digitakt *dt,
		struct digitakt_stream *stream)
{
	unsigned long flags;
	unsigned int pos;

	spin_lock_irqsave(&dt->lock, flags);
	pos = stream->buffer_pos;
	spin_unlock_irqrestore(&dt->lock, flags);
	return pos;
}

static snd_pcm_uframes_t capture_pcm_pointer(struct snd_pcm_substream *subs)
{
	struct digitakt *dt = subs->private_data;

	return digitakt_pcm_pointer(dt, &dt->capture);
}

static snd_pcm_uframes_t playback_pcm_pointer(struct snd_pcm_substream *subs)
{
	struct digitakt *dt = subs->private_data;

	return digitakt_pcm_pointer(dt, &dt->playback);
}

static const struct snd_pcm_ops capture_pcm_ops = {
	.open = capture_pcm_open,
	.close = capture_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = capture_pcm_hw_params,
	.hw_free = digitakt_pcm_hw_free,
	.prepare = capture_pcm_prepare,
	.trigger = capture_pcm_trigger,
	.pointer = capture_pcm_pointer,
	.page = snd_pcm_lib_get_vmalloc_page,
};

static const struct snd_pcm_ops playback_pcm_ops = {
	.open = playback_pcm_open,
	.close = playback_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = playback_pcm_hw_params,
	.hw_free = digitakt_pcm_hw_free,
	.prepare = playback_pcm_prepare,
	.trigger = playback_pcm_trigger,
	.pointer = playback_pcm_pointer,
	.page = snd_pcm_lib_get_vmalloc_page,
};

//static const struct uac_format_type_i_discrete_descriptor *
//find_format_descriptor(struct usb_interface *interface)
//{
//	struct usb_host_interface *alt;
//	u8 *extra;
//	int extralen;
//
//	if (interface->num_altsetting != 2) {
//		dev_err(&interface->dev, "invalid num_altsetting\n");
//		return NULL;
//	}
//
//	alt = &interface->altsetting[0];
//	if (alt->desc.bNumEndpoints != 0) {
//		dev_err(&interface->dev, "invalid bNumEndpoints\n");
//		return NULL;
//	}
//
//	alt = &interface->altsetting[1];
//	if (alt->desc.bNumEndpoints != 1) {
//		dev_err(&interface->dev, "invalid bNumEndpoints\n");
//		return NULL;
//	}
//
//	extra = alt->extra;
//	extralen = alt->extralen;
//	while (extralen >= sizeof(struct usb_descriptor_header)) {
//		struct uac_format_type_i_discrete_descriptor *desc;
//
//		desc = (struct uac_format_type_i_discrete_descriptor *)extra;
//		if (desc->bLength > extralen) {
//			dev_err(&interface->dev, "descriptor overflow\n");
//			return NULL;
//		}
//		if (desc->bLength == UAC_FORMAT_TYPE_I_DISCRETE_DESC_SIZE(1) &&
//		    desc->bDescriptorType == USB_DT_CS_INTERFACE &&
//		    desc->bDescriptorSubtype == UAC_FORMAT_TYPE) {
//			if (desc->bFormatType != UAC_FORMAT_TYPE_I_PCM ||
//			    desc->bSamFreqType != 1) {
//				dev_err(&interface->dev,
//					"invalid format type\n");
//				return NULL;
//			}
//			return desc;
//		}
//		extralen -= desc->bLength;
//		extra += desc->bLength;
//	}
//	dev_err(&interface->dev, "sample format descriptor not found\n");
//	return NULL;
//}

//static int detect_usb_format(struct digitakt *ua)
//{
//	const struct uac_format_type_i_discrete_descriptor *fmt_capture;
//	const struct uac_format_type_i_discrete_descriptor *fmt_playback;
//	const struct usb_endpoint_descriptor *epd;
//	unsigned int rate2;
//
//	fmt_capture = find_format_descriptor(ua->intf[INTF_CAPTURE]);
//	fmt_playback = find_format_descriptor(ua->intf[INTF_PLAYBACK]);
//	if (!fmt_capture || !fmt_playback)
//		return -ENXIO;
//
//	switch (fmt_capture->bSubframeSize) {
//	case 3:
//		ua->format_bit = SNDRV_PCM_FMTBIT_S24_3LE;
//		break;
//	case 4:
//		ua->format_bit = SNDRV_PCM_FMTBIT_S32_LE;
//		break;
//	default:
//		dev_err(&ua->dev->dev, "sample width is not 24 or 32 bits\n");
//		return -ENXIO;
//	}
//	if (fmt_capture->bSubframeSize != fmt_playback->bSubframeSize) {
//		dev_err(&ua->dev->dev,
//			"playback/capture sample widths do not match\n");
//		return -ENXIO;
//	}
//
//	if (fmt_capture->bBitResolution != 24 ||
//	    fmt_playback->bBitResolution != 24) {
//		dev_err(&ua->dev->dev, "sample width is not 24 bits\n");
//		return -ENXIO;
//	}
//
//	ua->rate = combine_triple(fmt_capture->tSamFreq[0]);
//	rate2 = combine_triple(fmt_playback->tSamFreq[0]);
//	if (ua->rate != rate2) {
//		dev_err(&ua->dev->dev,
//			"playback/capture rates do not match: %u/%u\n",
//			rate2, ua->rate);
//		return -ENXIO;
//	}
//
//	switch (ua->dev->speed) {
//	case USB_SPEED_FULL:
//		ua->packets_per_second = 1000;
//		break;
//	case USB_SPEED_HIGH:
//		ua->packets_per_second = 8000;
//		break;
//	default:
//		dev_err(&ua->dev->dev, "unknown device speed\n");
//		return -ENXIO;
//	}
//
//	ua->capture.channels = fmt_capture->bNrChannels;
//	ua->playback.channels = fmt_playback->bNrChannels;
//	ua->capture.frame_bytes =
//		fmt_capture->bSubframeSize * ua->capture.channels;
//	ua->playback.frame_bytes =
//		fmt_playback->bSubframeSize * ua->playback.channels;
//
//	epd = &ua->intf[INTF_CAPTURE]->altsetting[1].endpoint[0].desc;
//	if (!usb_endpoint_is_isoc_in(epd)) {
//		dev_err(&ua->dev->dev, "invalid capture endpoint\n");
//		return -ENXIO;
//	}
//	ua->capture.usb_pipe = usb_rcvisocpipe(ua->dev, usb_endpoint_num(epd));
//	ua->capture.max_packet_bytes = usb_endpoint_maxp(epd);
//
//	epd = &ua->intf[INTF_PLAYBACK]->altsetting[1].endpoint[0].desc;
//	if (!usb_endpoint_is_isoc_out(epd)) {
//		dev_err(&ua->dev->dev, "invalid playback endpoint\n");
//		return -ENXIO;
//	}
//	ua->playback.usb_pipe = usb_sndisocpipe(ua->dev, usb_endpoint_num(epd));
//	ua->playback.max_packet_bytes = usb_endpoint_maxp(epd);
//	return 0;
//}

static int alloc_stream_buffers(struct digitakt *dt,
		struct digitakt_stream *stream)
{
	unsigned int i;
	size_t size;

	stream->queue_length = queue_length;
	stream->queue_length = max(stream->queue_length,
				   (unsigned int)MIN_QUEUE_LENGTH);
	stream->queue_length = min(stream->queue_length,
				   (unsigned int)MAX_QUEUE_LENGTH);

	for (i = 0; i < ARRAY_SIZE(stream->buffers); ++i) {

		size = stream->max_packet_bytes;
		stream->buffers[i].addr =
			usb_alloc_coherent(dt->dev, size, GFP_KERNEL,
					   &stream->buffers[i].dma);
		if (!stream->buffers[i].addr) {
			snd_printd("alloc_stream_buffers(%lu) failed!", size);
			return -ENOMEM;
		} else {
			snd_printd("alloc_stream_buffers(%lu) OK!", size);
		}
		stream->buffers[i].size = size;
	}
	snd_printd("alloc_stream_buffers ok!");
	return 0;
}

static void free_stream_buffers(struct digitakt *dt,
		struct digitakt_stream *stream)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(stream->buffers); ++i)
		usb_free_coherent(dt->dev,
				  stream->buffers[i].size,
				  stream->buffers[i].addr,
				  stream->buffers[i].dma);
}

static int alloc_stream_urbs(struct digitakt *dt,
		struct digitakt_stream *stream,
			     void (*urb_complete)(struct urb *))
{
	unsigned max_packet_size = stream->max_packet_bytes;
	struct digitakt_urb *urb;
	unsigned int b = 0;

	snd_printd("stream->max_packet_bytes = %u", max_packet_size);

	for (b = 0; b < ARRAY_SIZE(stream->buffers); ++b) {
		unsigned int size = stream->buffers[b].size;
		u8 *addr = stream->buffers[b].addr;
		dma_addr_t dma = stream->buffers[b].dma;
		snd_printd("addr = %16X", addr);
		snd_printd("b = %u", b);
		snd_printd("size = %u", size);
		snd_printd("dma_addr = %16X", dma);
		urb = kmalloc(sizeof(*urb), GFP_KERNEL);
		if (!urb)
			return -ENOMEM;
		snd_printd("kmalloc ok!");
		usb_init_urb(&urb->urb);
		snd_printd("init_urb ok!");
		urb->urb.dev = dt->dev;
		urb->urb.pipe = stream->usb_pipe;
		urb->urb.transfer_flags = URB_NO_TRANSFER_DMA_MAP;
		urb->urb.transfer_buffer = addr;
		urb->urb.transfer_dma = dma;
		urb->urb.transfer_buffer_length = max_packet_size;
		urb->urb.number_of_packets = 7;
		urb->urb.interval = 1;
		urb->urb.context = dt;
		urb->urb.complete = urb_complete;
		//urb->urb.iso_frame_desc[0].offset = 0;
		//urb->urb.iso_frame_desc[0].length = max_packet_size;
		stream->urbs[b] = urb;
	}

	snd_printd("alloc_stream_urbs ok!");
	return 0;
}

static void free_stream_urbs(struct digitakt_stream *stream)
{
	unsigned int i;

	for (i = 0; i < stream->queue_length; ++i) {
		kfree(stream->urbs[i]);
		stream->urbs[i] = NULL;
	}
}

static void free_usb_related_resources(struct digitakt *dt,
				       struct usb_interface *interface)
{
	unsigned int i;
	struct usb_interface *intf;

	mutex_lock(&dt->mutex);
	free_stream_urbs(&dt->capture);
	free_stream_urbs(&dt->playback);
	mutex_unlock(&dt->mutex);
	free_stream_buffers(dt, &dt->capture);
	free_stream_buffers(dt, &dt->playback);

	for (i = 0; i < ARRAY_SIZE(dt->intf); ++i) {
		mutex_lock(&dt->mutex);
		intf = dt->intf[i];
		dt->intf[i] = NULL;
		mutex_unlock(&dt->mutex);
		if (intf) {
			usb_set_intfdata(intf, NULL);
			if (intf != interface)
				usb_driver_release_interface(&digitakt_driver,
							     intf);
		}
	}
}

static void digitakt_card_free(struct snd_card *card)
{
	struct digitakt *dt = card->private_data;

	mutex_destroy(&dt->mutex);
}

static int digitakt_probe(struct usb_interface *interface,
		       const struct usb_device_id *usb_id)
{
	static const struct snd_usb_midi_endpoint_info midi_ep = {
		.out_cables =
			0x0000, .in_cables = 0x0000, .in_ep = 0x81, .out_ep = 0x01
	};
	static const struct snd_usb_audio_quirk midi_quirk = {
		.type = QUIRK_MIDI_FIXED_ENDPOINT,
		.data = &midi_ep
	};

	struct snd_card *card;
	struct digitakt *dt;
	unsigned int card_index, i;

	const char *name;
	char usb_path[32];
	int err;
	snd_printd("hallotest");

	mutex_lock(&devices_mutex);

	for (card_index = 0; card_index < SNDRV_CARDS; ++card_index)
		if (enable[card_index] && !(devices_used & (1 << card_index)))
			break;
	if (card_index >= SNDRV_CARDS) {
		mutex_unlock(&devices_mutex);
		return -ENOENT;
	}
	err = snd_card_new(&interface->dev,
			   index[card_index], id[card_index], THIS_MODULE,
			   sizeof(*dt), &card);
	if (err < 0) {
		mutex_unlock(&devices_mutex);
		return err;
	}
	card->private_free = digitakt_card_free;
	dt = card->private_data;
	dt->dev = interface_to_usbdev(interface);
	dt->card = card;
	dt->card_index = card_index;
	INIT_LIST_HEAD(&dt->midi_list);
	spin_lock_init(&dt->lock);
	mutex_init(&dt->mutex);
	INIT_LIST_HEAD(&dt->ready_playback_urbs);
	tasklet_init(&dt->playback_tasklet,
		     playback_tasklet, (unsigned long)dt);
	init_waitqueue_head(&dt->alsa_capture_wait);
	init_waitqueue_head(&dt->rate_feedback_wait);
	init_waitqueue_head(&dt->alsa_playback_wait);

//	err = usb_driver_set_configuration(dt->dev, 1);
//	if (err < 0) {
//		goto probe_error;
//	}
	snd_printd("sclaim");
	dt->intf[0] = interface;
	for (i = 1; i < ARRAY_SIZE(dt->intf); ++i) {
		dt->intf[i] = usb_ifnum_to_if(dt->dev,
					      i);
		if (!dt->intf[i]) {
			dev_err(&dt->dev->dev, "interface %u not found\n",
				i);
			err = -ENXIO;
			goto probe_error;
		}
		err = usb_driver_claim_interface(&digitakt_driver,
						 dt->intf[i], dt);
		snd_printd("claim %i is %i", i, err);
		if (err < 0) {
			dt->intf[i] = NULL;
			err = -EBUSY;
			goto probe_error;
		}
	}
	snd_printd("eclaim");

	err = enable_alt_setting(dt, 2, 2);
	if (err < 0)
		goto probe_error;

	err = enable_alt_setting(dt, 1, 3);
	if (err < 0)
		goto probe_error;

	snd_printd("alt settings ok!");
	dt->rate = 48000;
	dt->capture.channels = 12;
	dt->playback.channels = 2;
// we're using int transfers only
	// and hardcode stuff ...
	dt->capture.usb_pipe = usb_rcvintpipe(dt->dev, 3);
	dt->capture.max_packet_bytes = 368 * 24;
	dt->playback.usb_pipe = usb_sndintpipe(dt->dev, 3);
	dt->playback.max_packet_bytes = 88 * 24;
	dt->format_bit = SNDRV_PCM_FMTBIT_S32_BE;
	dt->packets_per_second = 8000;

	name = "Digitakt";
	strcpy(card->driver, "Digitakt");
	strcpy(card->shortname, name);

	usb_make_path(dt->dev, usb_path, sizeof(usb_path));
	snprintf(dt->card->longname, sizeof(dt->card->longname),
			"Digitakt %s (serial %s), %u Hz at %s, %s speed", name,
		 dt->dev->serial ? dt->dev->serial : "?", dt->rate, usb_path,
		 dt->dev->speed == USB_SPEED_HIGH ? "high" : "full");

	err = alloc_stream_buffers(dt, &dt->capture);
	if (err < 0)
		goto probe_error;
	err = alloc_stream_buffers(dt, &dt->playback);
	if (err < 0)
		goto probe_error;

	snd_printd("alloc_stream_buffers ok!");

	err = alloc_stream_urbs(dt, &dt->capture, capture_urb_complete);
	if (err < 0)
		goto probe_error;
	snd_printd("alloc_stream_urb capture ok!");
	err = alloc_stream_urbs(dt, &dt->playback, playback_urb_complete);
	if (err < 0)
		goto probe_error;
	snd_printd("alloc_stream_urb playback ok!");
	err = snd_pcm_new(card, name, 0, 1, 1, &dt->pcm);
	if (err < 0)
		goto probe_error;
	dt->pcm->private_data = dt;
	strcpy(dt->pcm->name, name);
	snd_pcm_set_ops(dt->pcm, SNDRV_PCM_STREAM_PLAYBACK, &playback_pcm_ops);
	snd_pcm_set_ops(dt->pcm, SNDRV_PCM_STREAM_CAPTURE, &capture_pcm_ops);

	err = snd_usbmidi_create(card, dt->intf[3],
				 &dt->midi_list, &midi_quirk);
	if (err < 0)
		goto probe_error;

	err = snd_card_register(card);
	if (err < 0)
		goto probe_error;

	usb_set_intfdata(interface, dt);
	devices_used |= 1 << card_index;

	mutex_unlock(&devices_mutex);
	snd_printd("probe ok!");
	return 0;

probe_error:
	free_usb_related_resources(dt, interface);
	snd_card_free(card);
	mutex_unlock(&devices_mutex);
	return err;
}

static void digitakt_disconnect(struct usb_interface *interface)
{
	struct digitakt *dt = usb_get_intfdata(interface);
	struct list_head *midi;

	if (!dt)
		return;

	mutex_lock(&devices_mutex);

	set_bit(DISCONNECTED, &dt->states);
	wake_up(&dt->rate_feedback_wait);

	/* make sure that userspace cannot create new requests */
	snd_card_disconnect(dt->card);

	/* make sure that there are no pending USB requests */
	list_for_each(midi, &dt->midi_list)
		snd_usbmidi_disconnect(midi);
	abort_alsa_playback(dt);
	abort_alsa_capture(dt);
	mutex_lock(&dt->mutex);
	stop_usb_playback(dt);
	stop_usb_capture(dt);
	mutex_unlock(&dt->mutex);

	free_usb_related_resources(dt, interface);

	devices_used &= ~(1 << dt->card_index);

	snd_card_free_when_closed(dt->card);

	mutex_unlock(&devices_mutex);
}

static const struct usb_device_id digitakt_ids[] = {
	{ USB_DEVICE(0x1935, 0x000c) },
	{ }
};
MODULE_DEVICE_TABLE(usb, digitakt_ids);

static struct usb_driver digitakt_driver = {
	.name = "snd-digitakt",
	.id_table = digitakt_ids,
	.probe = digitakt_probe,
	.disconnect = digitakt_disconnect,
};

module_usb_driver(digitakt_driver);
