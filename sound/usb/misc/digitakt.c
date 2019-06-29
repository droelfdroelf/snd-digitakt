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

/*
 * a "default" transfer is 24 blocks long, each holding 7 samples
 * TBD: Can we decrease the length to reduce latency??
 */

#define MIN_TRANSFER_SIZE_BLOCKS	2
#define MAX_TRANSFER_SIZE_BLOCKS	24
#define DEFAULT_TRANSFER_SIZE_BLOCKS	24
#define MAX_MEMORY_BUFFERS	4

static unsigned int transfer_size_blocks = DEFAULT_TRANSFER_SIZE_BLOCKS;

// two output channels
#define DT_NUM_PLAYBACK_CHANS	2

// 8 sample channels, 2 fx channels, 2 ext in channels
#define DT_NUM_RECORD_CHANS	12
#define DT_RECORD_FRAME_BYTES	(DT_NUM_RECORD_CHANS * DT_BYTES_PER_SAMPLE)
#define DT_PLAYBACK_FRAME_BYTES	(DT_NUM_PLAYBACK_CHANS * DT_BYTES_PER_SAMPLE)
#define DT_SAMPLES_PER_BLOCK 7
#define DT_BYTES_PER_SAMPLE	4 // uint32_t BE
#define DT_SAMPLES_PER_URB	(transfer_size_blocks * DT_SAMPLES_PER_BLOCK )	// 168 samples long by default
#define DT_HEADER_SIZE_BYTES	32
#define DT_PLAYBACK_BLOCK_LEN_BYTES  ( DT_HEADER_SIZE_BYTES + ( DT_SAMPLES_PER_BLOCK * DT_BYTES_PER_SAMPLE * DT_NUM_PLAYBACK_CHANS))// usually 88
#define DT_RECORD_BLOCK_LEN_BYTES  ( DT_HEADER_SIZE_BYTES + ( DT_SAMPLES_PER_BLOCK * DT_BYTES_PER_SAMPLE * DT_NUM_RECORD_CHANS))// usually 386

#define TRANSFER_OUT_DATA_SIZE	(DT_PLAYBACK_BLOCK_LEN_BYTES * transfer_size_blocks) // usually 2112
#define TRANSFER_IN_DATA_SIZE (DT_RECORD_BLOCK_LEN_BYTES * transfer_size_blocks) // usually 8832
#define MIN(a,b) (((a)<(b))?(a):(b))
static int index[SNDRV_CARDS] = SNDRV_DEFAULT_IDX;
static char *id[SNDRV_CARDS] = SNDRV_DEFAULT_STR;
static bool enable[SNDRV_CARDS] = SNDRV_DEFAULT_ENABLE_PNP;


module_param_array(index, int, NULL, 0444);
MODULE_PARM_DESC(index, "card index");
module_param_array(id, charp, NULL, 0444);
MODULE_PARM_DESC(id, "ID string");
module_param_array(enable, bool, NULL, 0444);
MODULE_PARM_DESC(enable, "enable card");
module_param(transfer_size_blocks, int, 0644);
MODULE_PARM_DESC(transfer_size_blocks, "transfer size in blocks");

enum {
	INTF_PLAYBACK = 0,
	INTF_CAPTURE,
	INTF_UNKNOWN_A,
	INTF_MIDI,
	INTF_UNKNOWN_B,
	INTF_COUNT = 5
};

/* bits in struct digitakt::states */
enum {
	USB_PLAYBACK_RUNNING,
	USB_CAPTURE_RUNNING,
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
	unsigned int rate_feedback_count;
	struct list_head ready_playback_urbs;
	struct tasklet_struct playback_tasklet;
	wait_queue_head_t alsa_capture_wait;
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
			struct list_head ready_list;
		}*urbs[MAX_TRANSFER_SIZE_BLOCKS];
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
 * copies n frames of the record urb from src to dst with max mbuf buffers, starting at pos
 * returns true if wraparound occured
 */

static void cpfmurb(unsigned int n, void* srcurb, unsigned int pos_src,
		void* dstbuf, unsigned int pos_dst) {

	unsigned int frames_left;
	unsigned int field;
	unsigned int frame_in_field;
	unsigned int frames_to_next_header;
	unsigned int transfer_size;
	unsigned int transfer_size_frames;
	unsigned int pos_src_, pos_dst_;
	void* src;
	void* dst;
	pos_src_ = pos_src;
	pos_dst_ = pos_dst;

	frames_left = n;
	while (frames_left) {
		field = pos_src_ / DT_SAMPLES_PER_BLOCK;
		frame_in_field = pos_src_ % DT_SAMPLES_PER_BLOCK;
		frames_to_next_header = DT_SAMPLES_PER_BLOCK - frame_in_field;
		src = srcurb + (DT_HEADER_SIZE_BYTES * (1 + field))
				+ (pos_src_ * DT_RECORD_FRAME_BYTES);
		transfer_size_frames = (MIN(frames_to_next_header, frames_left));
		transfer_size = transfer_size_frames * DT_RECORD_FRAME_BYTES;
		dst = dstbuf + (pos_dst_ * DT_RECORD_FRAME_BYTES);
//		snd_printdd("cpfmurb: %px -> %px, %u", src, dst, transfer_size_frames);
		memcpy(dst, src, transfer_size);
		pos_dst_ += transfer_size_frames;
		pos_src_ += transfer_size_frames;
		if (transfer_size_frames > frames_left) {
			frames_left = 0;
			snd_printdd("cpfmurb WARNING - uneven buf size");
		} else {
			frames_left -= transfer_size_frames;
		}
	}
}

static void cptourb(unsigned int n_samples, void* srcbuf, void* dsturb,
		unsigned int start_pos_samples) {

	unsigned int samples_left;
	unsigned int pos_src, pos_dst_, pos_samples;
	void* src;
	void* dst;

	pos_src = 0;
	pos_dst_ = start_pos_samples;
	src = srcbuf;
	samples_left = n_samples;
	pos_samples = start_pos_samples;
	while (samples_left) {
		src = srcbuf + (pos_src * DT_PLAYBACK_FRAME_BYTES);
		dst = dsturb
				+ ((((pos_samples / 7) + 1) * 4) + pos_samples)
						* DT_PLAYBACK_FRAME_BYTES; // offset in samples due to headers
		memcpy(dst, src, DT_PLAYBACK_FRAME_BYTES);
		pos_dst_++;
		pos_src++;
		pos_samples++;
		samples_left--;
	}
}

/*
 *  fill in dummy timestamp in playback data
 */
static void fill_in_meta_playback(u8* data, unsigned int len) {
	static u16 dummy_timestamp = 0;
	unsigned int offs = 0;
	snd_printdd("fill_in_meta_playback");
	while (offs < len) {
		data[offs] = 0x07;
		data[offs + 1] = 0xFF;
		data[offs + 2] = (dummy_timestamp >> 8) & 0xFF;
		data[offs + 3] = (dummy_timestamp) & 0xFF;
		offs += DT_PLAYBACK_BLOCK_LEN_BYTES;	// block length, usually 88
		dummy_timestamp += DT_SAMPLES_PER_BLOCK;
	}
}

static void abort_usb_capture(struct digitakt *dt)
{
	snd_printdd("abort_usb_capture");
	if (test_and_clear_bit(USB_CAPTURE_RUNNING, &dt->states)) {
		wake_up(&dt->alsa_capture_wait);
	}
}

static void abort_usb_playback(struct digitakt *dt)
{
	snd_printdd("abort_usb_playback");
	if (test_and_clear_bit(USB_PLAYBACK_RUNNING, &dt->states))
		wake_up(&dt->alsa_playback_wait);
}

static void playback_urb_complete(struct urb *usb_urb)
{
	struct digitakt_urb *urb = (struct digitakt_urb *) usb_urb;
	struct digitakt *dt = urb->urb.context;
	unsigned long flags;
	snd_printdd("playback_urb_complete");
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
		dt->playback.substream->runtime->delay -= DT_SAMPLES_PER_URB;
		//	DT_SAMPLES_PER_URB;// todo: fix for variable number of packets per urb
		spin_unlock_irqrestore(&dt->lock, flags);
	}
}

static void first_playback_urb_complete(struct urb *urb)
{
	struct digitakt *dt = urb->context;
	snd_printdd("first_playback_urb_complete");
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
	void *source, *dst;
	static unsigned int last_buf_pos = 0;
	static unsigned int last_per_pos = 0;

	runtime = stream->substream->runtime;
	frame_bytes = stream->frame_bytes;
	//
	source = runtime->dma_area + stream->buffer_pos * frame_bytes;
	dst = urb->transfer_buffer;

	if (stream->buffer_pos + frames <= runtime->buffer_size) {
		cptourb(frames, source, dst, 0);
	} else {
		/* wrap around at end of ring buffer */

		//	frames);
		frames1 = runtime->buffer_size - stream->buffer_pos;
		snd_printdd("wrap frame_bytes: %u frames: %u frames1: %u", frame_bytes,
				frames, frames1);
		snd_printdd("wrap: %px -> %px", source, dst);

		cptourb(frames1, source, dst, 0);
		cptourb(frames - frames1, runtime->dma_area, dst, frames1);
	}

	snd_printdd("positions: b: %u %u p: %u %u", stream->buffer_pos,
			stream->buffer_pos - last_buf_pos, stream->period_pos,
			stream->period_pos - last_per_pos);
	last_buf_pos = stream->buffer_pos;
	last_per_pos = stream->period_pos;

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
	snd_printdd("playback tasklet");
	snd_printdd("fbcount: %u", dt->rate_feedback_count);
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
		/* take URB out of FIFO */
		urb = list_first_entry(&dt->ready_playback_urbs,
				struct digitakt_urb, ready_list);
		list_del(&urb->ready_list);
		dt->rate_feedback_count--;
		/* fill packet with data or silence */
		frames = DT_SAMPLES_PER_URB;
		if (test_bit(ALSA_PLAYBACK_RUNNING, &dt->states))
			do_period_elapsed |= copy_playback_data(&dt->playback,
								&urb->urb,
								frames);
		else
			memset(urb->urb.transfer_buffer, 0,
					(frames / DT_SAMPLES_PER_BLOCK)
							* DT_PLAYBACK_BLOCK_LEN_BYTES);

		// in any case, fill in header and dummy time stamp
		fill_in_meta_playback(urb->urb.transfer_buffer,
				(frames / DT_SAMPLES_PER_BLOCK) * DT_PLAYBACK_BLOCK_LEN_BYTES);

		/* and off you go ... */
		snd_printdd("usb_submit_urb");
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
	unsigned int frame_bytes, frames1;
	void *dest;
	void* src;

	runtime = stream->substream->runtime;
	frame_bytes = stream->frame_bytes;
	snd_printdd("copy capture data frame_bytes: %u frames: %u",
			frame_bytes,
			frames);
	dest = runtime->dma_area;
	src = urb->transfer_buffer;
	if (stream->buffer_pos + frames <= runtime->buffer_size) {
		cpfmurb(frames, src, 0, dest, stream->buffer_pos);
	} else {
		/* wrap around at end of ring buffer */
		frames1 = runtime->buffer_size - stream->buffer_pos;
		cpfmurb(frames1, src, 0, dest, stream->buffer_pos);
		cpfmurb((frames - frames1), src, frames1, dest, 0);
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
	unsigned int frames;
	bool do_period_elapsed;
	int err;
	snd_printdd("capture_urb_complete stat %i", urb->status);
	if (unlikely(urb->status == -ENOENT ||		/* unlinked */
		     urb->status == -ENODEV ||		/* device removed */
		     urb->status == -ECONNRESET ||	/* unlinked */
		     urb->status == -ESHUTDOWN))	/* device disabled */
		goto stream_stopped;

	if (urb->status >= 0) {
		//urb->actual_length /
		frames = DT_SAMPLES_PER_URB;	// again hard coded ...
	}
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
		dt->rate_feedback_count++;
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
	snd_printdd("first_capture_urb_complete");
	urb->complete = capture_urb_complete;
	capture_urb_complete(urb);

	set_bit(CAPTURE_URB_COMPLETED, &dt->states);
	wake_up(&dt->alsa_capture_wait);
}

static int submit_stream_urbs(struct digitakt *dt,
		struct digitakt_stream *stream)
{
	unsigned int i;
	snd_printdd("submit_stream_urbs");
	for (i = 0; i < stream->queue_length; i++) {
		// for (i = 0; i < 8; i++) {
		int err = usb_submit_urb(&stream->urbs[i]->urb, GFP_KERNEL);
		snd_printdd("i: %u", i);
		if (err < 0) {
			dev_err(&dt->dev->dev, "USB request error %d: %s\n",
				err, usb_error_string(err));
			return err;
		}
	}
	snd_printdd("submit_stream_urbs OK");
	return 0;
}

static void kill_stream_urbs(struct digitakt_stream *stream)
{
	unsigned int i;
	snd_printdd("kill_stream_urbs");
	for (i = 0; i < stream->queue_length; i++)
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

static void stop_usb_capture(struct digitakt *dt)
{
	snd_printdd("stop usb capture");
	clear_bit(USB_CAPTURE_RUNNING, &dt->states);

	kill_stream_urbs(&dt->capture);
}

static int start_usb_capture(struct digitakt *dt)
{
	int err;
	snd_printdd("start usb capture");
	if (test_bit(DISCONNECTED, &dt->states))
		return -ENODEV;

	if (test_bit(USB_CAPTURE_RUNNING, &dt->states))
		return 0;

	kill_stream_urbs(&dt->capture);
	dt->rate_feedback_count = 0;
	clear_bit(CAPTURE_URB_COMPLETED, &dt->states);
	dt->capture.urbs[0]->urb.complete = first_capture_urb_complete;

	set_bit(USB_CAPTURE_RUNNING, &dt->states);
	err = submit_stream_urbs(dt, &dt->capture);
	if (err < 0)
		stop_usb_capture(dt);
	if (!err)
		snd_printdd("start usb capture OK");
	return err;
}

static void stop_usb_playback(struct digitakt *dt)
{
	snd_printdd("sstop usb playback");
	clear_bit(USB_PLAYBACK_RUNNING, &dt->states);
	kill_stream_urbs(&dt->playback);
	tasklet_kill(&dt->playback_tasklet);
}

static int start_usb_playback(struct digitakt *dt)
{
	unsigned int i;
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
	snd_printdd("playback wof");

	if (test_bit(DISCONNECTED, &dt->states)) {
		stop_usb_playback(dt);
		return -ENODEV;
	}
	if (!test_bit(USB_CAPTURE_RUNNING, &dt->states)) {
		stop_usb_playback(dt);
		return -EIO;
	}

	for (i = 0; i < dt->playback.queue_length; i++) {
		/* all initial URBs contain silence */
		urb = &dt->playback.urbs[i]->urb;
		memset(urb->transfer_buffer, 0,
				transfer_size_blocks * DT_PLAYBACK_BLOCK_LEN_BYTES);
		fill_in_meta_playback(urb->transfer_buffer,
				transfer_size_blocks * DT_PLAYBACK_BLOCK_LEN_BYTES);
	}

	set_bit(USB_PLAYBACK_RUNNING, &dt->states);
	err = submit_stream_urbs(dt, &dt->playback);
	if (err < 0)
		stop_usb_playback(dt);
	if (!err)
		snd_printdd("start usb playback OK");
	return err;
}

static void abort_alsa_capture(struct digitakt *dt)
{
	snd_printdd("abort_alsa_capture");
	if (test_bit(ALSA_CAPTURE_RUNNING, &dt->states))
		snd_pcm_stop_xrun(dt->capture.substream);
}

static void abort_alsa_playback(struct digitakt *dt)
{
	snd_printdd("abort_alsa_playback");
	if (test_bit(ALSA_PLAYBACK_RUNNING, &dt->states))
		snd_pcm_stop_xrun(dt->playback.substream);
}

static int set_stream_hw(struct digitakt *dt,
		struct snd_pcm_substream *substream,
			 unsigned int channels)
{
	int err;
	snd_printdd("set_stream_hw");
	substream->runtime->hw.info =
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH |
		SNDRV_PCM_INFO_INTERLEAVED |
	SNDRV_PCM_INFO_BLOCK_TRANSFER;
	substream->runtime->hw.formats = dt->format_bit;
	substream->runtime->hw.rates = snd_pcm_rate_to_rate_bit(dt->rate);
	substream->runtime->hw.rate_min = dt->rate;
	substream->runtime->hw.rate_max = dt->rate;
	substream->runtime->hw.channels_min = channels;
	substream->runtime->hw.channels_max = channels;
	substream->runtime->hw.periods_min = 2;
	substream->runtime->hw.periods_max = UINT_MAX;
	// make sure to have even block boundaries in the buffers so we can copy whole blocks at once
	//substream->runtime->min_align = channels * DT_SAMPLES_PER_BLOCK * 4;
	substream->runtime->hw.buffer_bytes_max = 1024 * 1024 * 20;
	// for now we support only a fixed buffer size
	//substream->runtime->hw.period_bytes_min = 4 * channels
	//		* DT_SAMPLES_PER_URB;
	substream->runtime->hw.period_bytes_min = 64;
	snd_printdd("period_bytes_min: %lu",
			substream->runtime->hw.period_bytes_min);
	substream->runtime->hw.period_bytes_max = UINT_MAX;
	err = snd_pcm_hw_constraint_minmax(substream->runtime,
					   SNDRV_PCM_HW_PARAM_PERIOD_TIME,
					   1000, UINT_MAX); // 1000us is less than 64 samples at 48kHz

	snd_printdd("chans: %u err: %i", channels, err);
	if (err < 0)
		return err;
	err = snd_pcm_hw_constraint_msbits(substream->runtime, 0, 32, 24);
	snd_printdd("chans: %u err: %i", channels, err);
	if (!err) {
		snd_printdd("set_stream_hw OK");
	}
	return err;
}

static int pcm_open(struct snd_pcm_substream *substream)
{
	struct digitakt *dt = substream->private_data;
	int err;
	snd_printdd("pcm_open");
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dt->playback.substream = substream;
		err = set_stream_hw(dt, substream, dt->playback.channels);
	} else {
		dt->capture.substream = substream;
		err = set_stream_hw(dt, substream, dt->capture.channels);
//		substream->runtime->hw.fifo_size = DIV_ROUND_CLOSEST(dt->rate,
//				dt->packets_per_second);
//		substream->runtime->delay = substream->runtime->hw.fifo_size;

	}
	if (err < 0)
		return err;
	substream->runtime->hw.fifo_size = 0;
	substream->runtime->delay = 0;
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
		snd_printdd("pcm_open OK");
	}
	return err;
}

static int pcm_close(struct snd_pcm_substream *substream)
{
	struct digitakt *dt = substream->private_data;

	mutex_lock(&dt->mutex);
	stop_usb_playback(dt);
	clear_bit(ALSA_PLAYBACK_OPEN, &dt->states);
	if (!test_bit(ALSA_CAPTURE_OPEN, &dt->states))
		stop_usb_capture(dt);
	mutex_unlock(&dt->mutex);
	snd_printdd("pcm_close");
	return 0;
}


static int pcm_hw_params(struct snd_pcm_substream *substream,
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
	snd_printdd("pcm_hw_params");
	return snd_pcm_lib_alloc_vmalloc_buffer(substream,
						params_buffer_bytes(hw_params));
}

static int digitakt_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_vmalloc_buffer(substream);
}

static int pcm_prepare(struct snd_pcm_substream *substream)
{
	struct digitakt *dt = substream->private_data;
	int err;
	snd_printdd("pcm_prepare(..)");
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
	dt->capture.period_pos = 0;
	dt->capture.buffer_pos = 0;
	snd_printdd("pcm_prepare(..) OK");
	return 0;
}


static int pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct digitakt *dt = substream->private_data;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		snd_printdd("trig START!");
//		if (!test_bit(USB_PLAYBACK_RUNNING, &dt->states))
//			return -EIO;
		set_bit(ALSA_PLAYBACK_RUNNING, &dt->states);
		set_bit(ALSA_CAPTURE_RUNNING, &dt->states);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		snd_printdd("trig STOP!");
		clear_bit(ALSA_PLAYBACK_RUNNING, &dt->states);
		clear_bit(ALSA_CAPTURE_RUNNING, &dt->states);
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

static snd_pcm_uframes_t pcm_pointer(struct snd_pcm_substream *subs)
{
	struct digitakt *dt = subs->private_data;
	if (subs->stream == SNDRV_PCM_STREAM_PLAYBACK)
		return digitakt_pcm_pointer(dt, &dt->playback);
	else
		return digitakt_pcm_pointer(dt, &dt->capture);
}

static const struct snd_pcm_ops pcm_ops = {
	.open = pcm_open,
	.close = pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = pcm_hw_params,
	.hw_free = digitakt_pcm_hw_free,
	.prepare = pcm_prepare,
	.trigger = pcm_trigger,
	.pointer = pcm_pointer,
	.page = snd_pcm_lib_get_vmalloc_page,
};

static int alloc_stream_buffers(struct digitakt *dt,
		struct digitakt_stream *stream)
{
	unsigned int i;
	size_t size;
	stream->queue_length = MAX_MEMORY_BUFFERS;
	for (i = 0; i < ARRAY_SIZE(stream->buffers); i++) {

		size = stream->max_packet_bytes;
		stream->buffers[i].addr =
			usb_alloc_coherent(dt->dev, size, GFP_KERNEL,
					   &stream->buffers[i].dma);
		if (!stream->buffers[i].addr) {
			snd_printdd("alloc_stream_buffers(%lu) failed!", size);
			return -ENOMEM;
		} else {
			snd_printdd("alloc_stream_buffers(%lu) OK!", size);
		}
		stream->buffers[i].size = size;
	}
	snd_printdd("alloc_stream_buffers ok!");
	return 0;
}

static void free_stream_buffers(struct digitakt *dt,
		struct digitakt_stream *stream)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(stream->buffers); i++)
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

	snd_printdd("stream->max_packet_bytes = %u", max_packet_size);

	for (b = 0; b < ARRAY_SIZE(stream->buffers); b++) {
		unsigned int size = stream->buffers[b].size;
		u8 *addr = stream->buffers[b].addr;
		dma_addr_t dma = stream->buffers[b].dma;
		snd_printdd("addr = %llX", (long long unsigned int )addr);
		snd_printdd("b = %u", b);
		snd_printdd("size = %u", size);
		snd_printdd("dma_addr = %llX", dma);
		urb = kmalloc(sizeof(*urb), GFP_KERNEL);
		if (!urb)
			return -ENOMEM;
		snd_printdd("kmalloc ok!");
		usb_init_urb(&urb->urb);
		snd_printdd("init_urb ok!");
		urb->urb.dev = dt->dev;
		urb->urb.pipe = stream->usb_pipe;
		urb->urb.transfer_flags = URB_NO_TRANSFER_DMA_MAP;
		urb->urb.transfer_buffer = addr;
		urb->urb.transfer_dma = dma;
		urb->urb.transfer_buffer_length = max_packet_size;
		// urb->urb.number_of_packets = 7;
		urb->urb.interval = 1;
		urb->urb.context = dt;
		urb->urb.complete = urb_complete;
		//urb->urb.iso_frame_desc[0].offset = 0;
		//urb->urb.iso_frame_desc[0].length = max_packet_size;
		stream->urbs[b] = urb;
	}

	snd_printdd("alloc_stream_urbs ok!");
	return 0;
}

static void free_stream_urbs(struct digitakt_stream *stream)
{
	unsigned int i;

	for (i = 0; i < stream->queue_length; i++) {
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

	for (i = 0; i < ARRAY_SIZE(dt->intf); i++) {
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
			0x0001, .in_cables = 0x0001, .in_ep = 0x81, .out_ep = 0x01,
			.in_interval = 0, .out_interval = 0// 0 is for bulk transfer in midi.c
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
	snd_printdd("hallotest");

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
	init_waitqueue_head(&dt->alsa_playback_wait);

	snd_printdd("sclaim");
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
		snd_printdd("claim %i is %i", i, err);
		if (err < 0) {
			dt->intf[i] = NULL;
			err = -EBUSY;
			goto probe_error;
		}
	}
	snd_printdd("eclaim");

	err = enable_alt_setting(dt, 2, 2);
	if (err < 0)
		goto probe_error;

	err = enable_alt_setting(dt, 1, 3);
	if (err < 0)
		goto probe_error;

	snd_printdd("alt settings ok!");
	dt->rate = 48000;
	dt->capture.channels = 12;
	dt->playback.channels = 2;
	// we're using int transfers only
	dt->capture.usb_pipe = usb_rcvintpipe(dt->dev, 3);
	dt->capture.max_packet_bytes = DT_RECORD_BLOCK_LEN_BYTES
			* transfer_size_blocks;
	dt->playback.usb_pipe = usb_sndintpipe(dt->dev, 3);
	dt->playback.max_packet_bytes = DT_PLAYBACK_BLOCK_LEN_BYTES
			* transfer_size_blocks;
	dt->format_bit = SNDRV_PCM_FMTBIT_S32_BE;
	dt->packets_per_second = 286;

	dt->playback.frame_bytes = 4 * dt->playback.channels;
	dt->capture.frame_bytes = 4 * dt->capture.channels;

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

	snd_printdd("alloc_stream_buffers ok!");

	err = alloc_stream_urbs(dt, &dt->capture, capture_urb_complete);
	if (err < 0)
		goto probe_error;
	snd_printdd("alloc_stream_urb capture ok!");
	err = alloc_stream_urbs(dt, &dt->playback, playback_urb_complete);
	if (err < 0)
		goto probe_error;
	snd_printdd("alloc_stream_urb playback ok!");
	err = snd_pcm_new(card, name, 0, 1, 1, &dt->pcm);
	if (err < 0)
		goto probe_error;
	dt->pcm->private_data = dt;
	strcpy(dt->pcm->name, name);
	snd_pcm_set_ops(dt->pcm, SNDRV_PCM_STREAM_PLAYBACK, &pcm_ops);
	snd_pcm_set_ops(dt->pcm, SNDRV_PCM_STREAM_CAPTURE, &pcm_ops);

	err = snd_usbmidi_create(card, dt->intf[INTF_MIDI],
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
	snd_printd("Transfer size %u blocks (%u samples, period %u us @48kHz)",
			transfer_size_blocks, DT_SAMPLES_PER_URB,
			(DT_SAMPLES_PER_URB * 1000000) / 48000);
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
