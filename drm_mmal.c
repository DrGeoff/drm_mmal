/*
 * Forked from 6by9/drm_mmal
 *
 * I modified the original code to take the video stream from the Pi camera.
 */
/*
Copyright (c) 2017 Raspberry Pi (Trading) Ltd

Based on example_basic_2.c from the Userland repo (interface/mmal/test/examples)

Amended to use GEM to allocate the video buffers from CMA, export as dmabufs,
import the dmabufs via vcsm, and then use them with MMAL.
Output frames are then presented to libdrm for display.

Plays an H264 elementary stream with no frame scheduling.

Copyright (c) 2012, Broadcom Europe Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <drm.h>
#include <drm_mode.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <interface/vcsm/user-vcsm.h>
#include "bcm_host.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_queue.h"
#include "interface/vcos/vcos.h"
#include <stdio.h>

//Tested working:
//MMAL_ENCODING_I420 (YU12)
//MMAL_ENCODING_YV12
//MMAL_ENCODING_NV12
//MMAL_ENCODING_I422 (YU16)
//MMAL_ENCODING_BGRA
//MMAL_ENCODING_RGBA
//MMAL_ENCODING_RGB16 (RGB565)
//
//Valid to the ISP, but not DRM:
//MMAL_ENCODING_NV21 (NV12 with cb/cr swapped)
//MMAL_ENCODING_RGB24
//MMAL_ENCODING_BGR24
//Patches sorted for vc4_plane.c for each of these, and then they work.
//
#define ENCODING_FOR_DRM  MMAL_ENCODING_I420

#define DRM_MODULE "vc4"
#define MAX_BUFFERS 2

static inline int warn(const char *file, int line, const char *fmt, ...)
{
   int errsv = errno;
   va_list va;
   va_start(va, fmt);
   fprintf(stderr, "WARN(%s:%d): ", file, line);
   vfprintf(stderr, fmt, va);
   va_end(va);
   errno = errsv;
   fprintf(stderr, "\n");
   return 1;
}

#define CHECK_CONDITION(cond, ...) \
do { \
   if (cond) { \
      int errsv = errno; \
      fprintf(stderr, "ERROR(%s:%d) : ", \
         __FILE__, __LINE__); \
      errno = errsv; \
      fprintf(stderr,  __VA_ARGS__); \
      abort(); \
   } \
} while(0)
#define WARN_ON(cond, ...) \
   ((cond) ? warn(__FILE__, __LINE__, __VA_ARGS__) : 0)
#define ERRSTR strerror(errno)
#define CHECK_STATUS(status, ...) WARN_ON(status != MMAL_SUCCESS, __VA_ARGS__); \
   if (status != MMAL_SUCCESS) goto error;

static uint8_t codec_header_bytes[512];
static unsigned int codec_header_bytes_size = sizeof(codec_header_bytes);

static FILE *source_file;

/* Macros abstracting the I/O, just to make the example code clearer */
#define SOURCE_OPEN(uri) \
   source_file = fopen(uri, "rb"); if (!source_file) goto error;
#define SOURCE_READ_CODEC_CONFIG_DATA(bytes, size) \
   size = fread(bytes, 1, size, source_file); rewind(source_file)
#define SOURCE_READ_DATA_INTO_BUFFER(a) \
   a->length = fread(a->data, 1, a->alloc_size - 128, source_file); \
   a->offset = 0
#define SOURCE_CLOSE() \
   if (source_file) fclose(source_file)

/** Context for our application */
static struct CONTEXT_T {
   VCOS_SEMAPHORE_T semaphore;
   MMAL_QUEUE_T *queue;
   MMAL_STATUS_T status;
} context;

struct buffer {
   unsigned int bo_handle;
   unsigned int fb_handle;
   int dbuf_fd;
   unsigned int vcsm_handle;
   MMAL_BUFFER_HEADER_T *mmal_buffer;
};

struct drm_setup {
   int conId;
   uint32_t crtcId;
   int crtcIdx;
   uint32_t planeId;
   unsigned int out_fourcc;
   MMAL_RECT_T compose;
};


uint32_t mmal_encoding_to_drm_fourcc(uint32_t mmal_encoding)
{
   switch(mmal_encoding)
   {
      case MMAL_ENCODING_I420:
         return MMAL_FOURCC('Y','U','1','2');
      case MMAL_ENCODING_YV12:
         return MMAL_FOURCC('Y','V','1','2');
      case MMAL_ENCODING_I422:
         return MMAL_FOURCC('Y','U','1','6');
      case MMAL_ENCODING_NV12:
         return MMAL_FOURCC('N','V','1','2');
      case MMAL_ENCODING_NV21:
         return MMAL_FOURCC('N','V','2','1');
      case MMAL_ENCODING_RGB16:
         return MMAL_FOURCC('R','G','1','6');
      case MMAL_ENCODING_RGB24:
         return MMAL_FOURCC('B','G','2','4');
      case MMAL_ENCODING_BGR24:
         return MMAL_FOURCC('R','G','2','4');
      case MMAL_ENCODING_BGR32:
      case MMAL_ENCODING_BGRA:
         return MMAL_FOURCC('X','R','2','4');
      case MMAL_ENCODING_RGB32:
      case MMAL_ENCODING_RGBA:
         return MMAL_FOURCC('X','B','2','4');
      case MMAL_ENCODING_OPAQUE:
         fprintf(stderr, "MMAL_ENCODING_OPAQUE can't be converted to a DRM compatible format\n");
      default:
         return 0;
   }
}

void mmal_format_to_drm_pitches_offsets(uint32_t *pitches, uint32_t *offsets, uint32_t *bo_handles, MMAL_ES_FORMAT_T *format)
{
   switch (format->encoding)
   {
      // 3 plane YUV formats
      case MMAL_ENCODING_I420:
      case MMAL_ENCODING_YV12:
         pitches[0] = mmal_encoding_width_to_stride(format->encoding, format->es->video.width);
         pitches[1] = pitches[0] / 2;
         pitches[2] = pitches[1];

         offsets[1] = pitches[0] * format->es->video.height;
         offsets[2] = offsets[1] + pitches[1] * format->es->video.height/2;

         bo_handles[1] = bo_handles[2] = bo_handles[0];
         break;
      case MMAL_ENCODING_I422:
         pitches[0] = mmal_encoding_width_to_stride(format->encoding, format->es->video.width);
         pitches[1] = pitches[0] / 2;
         pitches[2] = pitches[1];

         offsets[1] = pitches[0] * format->es->video.height;
         offsets[2] = offsets[1] + pitches[1] * format->es->video.height;

         bo_handles[1] = bo_handles[2] = bo_handles[0];
         break;
      // 2 plane YUV formats
      case MMAL_ENCODING_NV12:
      case MMAL_ENCODING_NV21:
         pitches[0] = mmal_encoding_width_to_stride(format->encoding, format->es->video.width);
         pitches[1] = pitches[0];

         offsets[1] = pitches[0] * format->es->video.height;

         bo_handles[1] = bo_handles[0];
         break;
      default:
         pitches[0] = mmal_encoding_width_to_stride(format->encoding, format->es->video.width);
         break;
   }
   printf("MMAL format %08x translates to pitches [%u,%u,%u,%u], and offsets [%u,%u,%u,%u]\n",
            format->encoding,
            pitches[0],pitches[1],pitches[2],pitches[3],
            offsets[0],offsets[1],offsets[2],offsets[3]
            );
}

static int buffer_create(struct buffer *b, int drmfd, MMAL_PORT_T *port)
{
   struct drm_mode_create_dumb gem;
   struct drm_mode_destroy_dumb gem_destroy;
   int ret;

   memset(&gem, 0, sizeof gem);
   gem.width = port->format->es->video.width;
   gem.height = port->format->es->video.height;
   gem.bpp = 32;
   gem.size = port->buffer_size;
   ret = ioctl(drmfd, DRM_IOCTL_MODE_CREATE_DUMB, &gem);
   if (ret)
   {
      printf("CREATE_DUMB failed: %s\n", ERRSTR);
      return -1;
   }
   printf("SUCCESS: CREATE_DUMB\n");
   printf("bo %u %ux%u bpp %u size %lu (%u)\n", gem.handle, gem.width, gem.height, gem.bpp, (long)gem.size, port->buffer_size);
   b->bo_handle = gem.handle;

   struct drm_prime_handle prime;
   memset(&prime, 0, sizeof prime);
   prime.handle = b->bo_handle;

   ret = ioctl(drmfd, DRM_IOCTL_PRIME_HANDLE_TO_FD, &prime);
   if (ret)
   {
      printf("PRIME_HANDLE_TO_FD failed: %s\n", ERRSTR);
      goto fail_gem;
   }
   printf("dbuf_fd = %d\n", prime.fd);
   b->dbuf_fd = prime.fd;

   uint32_t offsets[4] = { 0 };
   uint32_t pitches[4] = { 0 };
   uint32_t bo_handles[4] = { b->bo_handle };
   unsigned int fourcc = mmal_encoding_to_drm_fourcc(port->format->encoding);

   mmal_format_to_drm_pitches_offsets(pitches, offsets, bo_handles, port->format);

   fprintf(stderr, "FB fourcc %c%c%c%c\n",
      fourcc,
      fourcc >> 8,
      fourcc >> 16,
      fourcc >> 24);

   b->vcsm_handle = vcsm_import_dmabuf(b->dbuf_fd, "DRM Buf");
   if (!b->vcsm_handle)
      goto fail_prime;

   ret = drmModeAddFB2(drmfd, port->format->es->video.crop.width,
      port->format->es->video.crop.height, fourcc, bo_handles,
      pitches, offsets, &b->fb_handle, 0);
   if (ret)
   {
      printf("drmModeAddFB2 failed: %s\n", ERRSTR);
      goto fail_vcsm;
   }

   printf("SUCCESS: buffer_create\n");
   return 0;

fail_vcsm:
   vcsm_free(b->vcsm_handle);

fail_prime:
   close(b->dbuf_fd);

fail_gem:
   memset(&gem_destroy, 0, sizeof gem_destroy);
   gem_destroy.handle = b->bo_handle,
   ret = ioctl(drmfd, DRM_IOCTL_MODE_DESTROY_DUMB, &gem_destroy);
   if (ret)
   {
      printf("DESTROY_DUMB failed: %s\n", ERRSTR);
   }

   return -1;
}

MMAL_POOL_T* pool_create_drm(MMAL_PORT_T *port, struct buffer *buffers, int drmfd)
{
   MMAL_POOL_T *pool;
   unsigned int i;

   pool = mmal_port_pool_create(port,
                                port->buffer_num,
                                0);

   for (i = 0; i < port->buffer_num; i++)
   {
      buffer_create(&buffers[i], drmfd, port);
      
      pool->header[i]->data = (uint8_t*)vcsm_vc_hdl_from_hdl(buffers[i].vcsm_handle);
      pool->header[i]->alloc_size = port->buffer_size;
      pool->header[i]->length = 0;
      buffers[i].mmal_buffer = pool->header[i];
   }

   printf("SUCCESS: pool_create_drm\n");
   return pool;
}

void buffer_destroy(int drmfd, struct buffer *buf)
{
   struct drm_mode_destroy_dumb gem_destroy;

   vcsm_free(buf->vcsm_handle);

   close(buf->dbuf_fd);

   memset(&gem_destroy, 0, sizeof gem_destroy);
   gem_destroy.handle = buf->bo_handle;
   ioctl(drmfd, DRM_IOCTL_MODE_DESTROY_DUMB, &gem_destroy);
}

void pool_destroy_drm(MMAL_PORT_T *port, MMAL_POOL_T *pool, struct buffer *buffers, int drmfd)
{
   unsigned int i;

   for (i = 0; i < pool->headers_num; i++)
   {
      buffer_destroy(drmfd, &buffers[i]);
   }
   mmal_port_pool_destroy(port, pool);
}

/** Callback from the control port.
 * Component is sending us an event. */
static void control_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)port->userdata;

   switch (buffer->cmd)
   {
   case MMAL_EVENT_EOS:
      /* Only sink component generate EOS events */
      break;
   case MMAL_EVENT_ERROR:
      /* Something went wrong. Signal this to the application */
      ctx->status = *(MMAL_STATUS_T *)buffer->data;
      break;
   default:
      break;
   }

   /* Done with the event, recycle it */
   mmal_buffer_header_release(buffer);

   /* Kick the processing thread */
   vcos_semaphore_post(&ctx->semaphore);
}

static int find_crtc(int drmfd, struct drm_setup *s, uint32_t *con)
{
   int ret = -1;
   int i;
   drmModeRes *res = drmModeGetResources(drmfd);
   if(!res) 
   {
      printf( "drmModeGetResources failed: %s\n", ERRSTR);
      return -1;
   }

   if (res->count_crtcs <= 0)
   {
      printf( "drm: no crts\n");
      goto fail_res;
   }

   if (!s->conId) {
      fprintf(stderr,
         "No connector ID specified.  Choosing default from list:\n");

      for (i = 0; i < res->count_connectors; i++) {
         drmModeConnector *con =
            drmModeGetConnector(drmfd, res->connectors[i]);
         drmModeEncoder *enc = NULL;
         drmModeCrtc *crtc = NULL;

         if (con->encoder_id) {
            enc = drmModeGetEncoder(drmfd, con->encoder_id);
            if (enc->crtc_id) {
               crtc = drmModeGetCrtc(drmfd, enc->crtc_id);
            }
         }

         if (!s->conId && crtc) {
            s->conId = con->connector_id;
            s->crtcId = crtc->crtc_id;
         }

         printf("Connector %d (crtc %d): type %d, %dx%d%s\n",
                con->connector_id,
                crtc ? crtc->crtc_id : 0,
                con->connector_type,
                crtc ? crtc->width : 0,
                crtc ? crtc->height : 0,
                (s->conId == (int)con->connector_id ?
            " (chosen)" : ""));
      }

      if (!s->conId) {
         fprintf(stderr,
            "No suitable enabled connector found.\n");
         exit(1);
      }
   }

   s->crtcIdx = -1;

   for (i = 0; i < res->count_crtcs; ++i) {
      if (s->crtcId == res->crtcs[i]) {
         s->crtcIdx = i;
         break;
      }
   }

   if (WARN_ON(s->crtcIdx == -1, "drm: CRTC %u not found\n", s->crtcId))
      goto fail_res;

   if (WARN_ON(res->count_connectors <= 0, "drm: no connectors\n"))
      goto fail_res;

   drmModeConnector *c;
   c = drmModeGetConnector(drmfd, s->conId);
   if (WARN_ON(!c, "drmModeGetConnector failed: %s\n", ERRSTR))
      goto fail_res;

   if (WARN_ON(!c->count_modes, "connector supports no mode\n"))
      goto fail_conn;

   {
      drmModeCrtc *crtc = drmModeGetCrtc(drmfd, s->crtcId);
      s->compose.x = crtc->x;
      s->compose.y = crtc->y;
      s->compose.width = crtc->width;
      s->compose.height = crtc->height;
      drmModeFreeCrtc(crtc);
   }

   if (con)
      *con = c->connector_id;
   ret = 0;

fail_conn:
   drmModeFreeConnector(c);

fail_res:
   drmModeFreeResources(res);

   return ret;
}

static int find_plane(int drmfd, struct drm_setup *s)
{
   drmModePlaneResPtr planes;
   drmModePlanePtr plane;
   unsigned int i;
   unsigned int j;
   int ret = 0;

   planes = drmModeGetPlaneResources(drmfd);
   if (WARN_ON(!planes, "drmModeGetPlaneResources failed: %s\n", ERRSTR))
      return -1;

   for (i = 0; i < planes->count_planes; ++i) {
      plane = drmModeGetPlane(drmfd, planes->planes[i]);
      if (WARN_ON(!planes, "drmModeGetPlane failed: %s\n", ERRSTR))
         break;

      if (!(plane->possible_crtcs & (1 << s->crtcIdx))) {
         drmModeFreePlane(plane);
         continue;
      }

      for (j = 0; j < plane->count_formats; ++j) {
         if (plane->formats[j] == s->out_fourcc)
            break;
      }

      if (j == plane->count_formats) {
         drmModeFreePlane(plane);
         continue;
      }

      s->planeId = plane->plane_id;
      drmModeFreePlane(plane);
      break;
   }

   if (i == planes->count_planes)
      ret = -1;

   drmModeFreePlaneResources(planes);
   return ret;
}


/** Callback from the output port.
 * Buffer has been produced by the port and is available for processing. */
static void output_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
   struct CONTEXT_T *ctx = (struct CONTEXT_T *)port->userdata;
   /* Queue the decoded video frame */
   fprintf(stderr, "Buffer %p returned cmd %08X, length %u\n", buffer, buffer->cmd, buffer->length);
   mmal_queue_put(ctx->queue, buffer);

   /* Kick the processing thread */
   vcos_semaphore_post(&ctx->semaphore);
}

static int drm_mmal_create_buffers(MMAL_PORT_T *port, struct buffer *buffers, int drmfd, MMAL_POOL_T **pool_out)
{
   printf("Entering: drm_mmal_create_buffers\n");
   MMAL_STATUS_T status;
   port->buffer_size = port->buffer_size_min;
   printf("Set port buffer size to %d\n", port->buffer_size);
   status = mmal_port_format_commit(port);

   *pool_out = pool_create_drm(port,
                              buffers, drmfd);

   // Note. buffer_num must be set before you get here 
   // 6by9: "The GPU allocates on port enable, so any changes afterwards are going to be bad news."
   printf("About to enable port %s\n", port->name);
   status = mmal_port_enable(port, output_callback);
   CHECK_STATUS(status, "failed to enable port");
   printf("SUCCESS: drm_mmal_create_buffers\n");
   return 0;
error:
   return status;
}

static int drm_mmal_destroy_buffers(MMAL_PORT_T *port, struct buffer *buffers, int drmfd, MMAL_POOL_T *pool_out)
{
   MMAL_STATUS_T status;

   status = mmal_port_disable(port);
   CHECK_STATUS(status, "failed to disable port");

   //Clear the queue of all buffers
   while(mmal_queue_length(pool_out->queue) != pool_out->headers_num)
   {
      MMAL_BUFFER_HEADER_T *buf;
      fprintf(stderr, "Wait for buffers to be returned. Have %d of %d buffers\n",
            mmal_queue_length(pool_out->queue), pool_out->headers_num);
      vcos_semaphore_wait(&context.semaphore);
      fprintf(stderr, "Got semaphore\n");
      buf = mmal_queue_get(context.queue);
      printf("Retrieved buf %p\n", buf);
      if (buf)
         mmal_buffer_header_release(buf);
   }
   fprintf(stderr, "Got all buffers\n");

   pool_destroy_drm(port, pool_out, buffers, drmfd);
   return 0;
error:
   return status;
}

static int setup_camera_config(MMAL_COMPONENT_T* camera, const int Width, const int Height)
{
    MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
        { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
        .max_stills_w = Width,
        .max_stills_h = Height,
        .stills_yuv422 = 0,
        .one_shot_stills = 0,
        .max_preview_video_w = Width,
        .max_preview_video_h = Height,
        .num_preview_video_frames = 3,
        .stills_capture_circular_buffer_height = 0,
        .fast_preview_resume = 0,
        .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
    };

    MMAL_STATUS_T status = mmal_port_parameter_set(camera->control, &cam_config.hdr);
    CHECK_STATUS(status, "failed to set camera config");

error:
   return status;
}


static int setup_port_format(MMAL_PORT_T* port, const int Width, const int Height)
{
   port->userdata = (void *)&context;
   port->buffer_num = 3;
   //camera->output[output_port]->buffer_size = camera->output[output_port]->buffer_size_recommended;

   MMAL_ES_FORMAT_T *format_out = port->format;

   //format_out->encoding = MMAL_ENCODING_OPAQUE;
   format_out->encoding = ENCODING_FOR_DRM;
   format_out->encoding_variant = MMAL_ENCODING_I420;
    format_out->es->video.width = VCOS_ALIGN_UP(Width,32);
    format_out->es->video.height = VCOS_ALIGN_UP(Height,16);
    format_out->es->video.crop.x = 0;
    format_out->es->video.crop.y = 0;
    format_out->es->video.crop.width = Width;
    format_out->es->video.crop.height = Height;
    format_out->es->video.frame_rate.num = 0;  // 0 means variable frame rate
    format_out->es->video.frame_rate.den = 1;

   MMAL_STATUS_T status = mmal_port_format_commit(port);
   CHECK_STATUS(status, "failed to commit format");

   /* Display the output port format */
   fprintf(stderr, "%s\n", port->name);
   fprintf(stderr, " type: %i, fourcc: %4.4s\n", format_out->type, (char *)&format_out->encoding);
   fprintf(stderr, " bitrate: %i, framed: %i\n", format_out->bitrate,
           !!(format_out->flags & MMAL_ES_FORMAT_FLAG_FRAMED));
   fprintf(stderr, " extra data: %i, %p\n", format_out->extradata_size, format_out->extradata);
   fprintf(stderr, " width: %i, height: %i, (%i,%i,%i,%i)\n",
           format_out->es->video.width, format_out->es->video.height,
           format_out->es->video.crop.x, format_out->es->video.crop.y,
           format_out->es->video.crop.width, format_out->es->video.crop.height);

error:
   return status;
}

int main(int argc, char **argv)
{
   const int Width = 3280;
   //const int Width = 1920;
   const int Height = 2464;
   //const int Height = 1080;

   MMAL_STATUS_T status = MMAL_EINVAL;
   MMAL_COMPONENT_T *camera = NULL;
   MMAL_COMPONENT_T* null_sink_component = NULL;
   MMAL_CONNECTION_T* preview_connection = NULL;  // Only used to connect preview to the null sink
   MMAL_POOL_T *pool_out = NULL;
   MMAL_BOOL_T eos_sent = MMAL_FALSE, eos_received = MMAL_FALSE;
   struct drm_setup setup = {0};
   struct buffer buffers[MAX_BUFFERS];
   MMAL_BUFFER_HEADER_T *current_buffer = NULL;
   unsigned int in_count = 0, conn_out_count = 0, conn_in_count = 0, out_count = 0;
   int ret;

   // Usage ./drm_mmal [0,1,2]
   if (argc < 2)
   {
      fprintf(stderr, "Usage: ./drm_mmal [0,1,2]  # Specify which camera port to use\n");
      return -1;
   }
   const int output_port = atoi(argv[1]);  // 0 == Preview, 1 == Video stream, 2 == Still camera.

   bcm_host_init();
   printf("SUCCESS: bcm_host_init\n");

   int drmfd = drmOpen(DRM_MODULE, NULL);
   CHECK_CONDITION(drmfd < 0, "drmOpen(%s) failed: %s\n", DRM_MODULE, ERRSTR);

   vcsm_init();
   printf("SUCCESS: vcsm_init\n");

   vcos_semaphore_create(&context.semaphore, "example", 1);
   printf("SUCCESS: vcos_semaphore_create\n");

   status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
   CHECK_STATUS(status, "failed to create camera");

#if 0
   /* Enable control port so we can receive events from the component */
   camera->control->userdata = (void *)&context;
   status = mmal_port_enable(camera->control, control_callback);
   CHECK_STATUS(status, "failed to enable control port");
#endif

   setup_camera_config(camera, Width, Height);

   /* Set the zero-copy parameter on all camera ports */
   for (int index = 0; index !=3; ++index)
   {
      status = mmal_port_parameter_set_boolean(camera->output[index], MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
      CHECK_STATUS(status, "failed to set zero copy - %s", camera->output[output_port]->name);
   }

   setup_port_format(camera->output[0], Width, Height);
   if (output_port != 0)
   {
      setup_port_format(camera->output[output_port], Width, Height);
   }

   /* Create a queue to store our decoded video frames. The callback we will get when
    * a frame has been decoded will put the frame into this queue. */
   context.queue = mmal_queue_create();

   /* Component won't start processing data until it is enabled. */
   status = mmal_component_enable(camera);
   CHECK_STATUS(status, "failed to enable camera component");

   // If the output_port is not the preview port then connect the preview port to the null sink
   // The camera preview port must be connected to something for calculating exposure and white balance settings.

   if (output_port != 0)
   {
      status = mmal_port_parameter_set_boolean(camera->output[output_port], MMAL_PARAMETER_CAPTURE, 1);
      CHECK_STATUS(status, "Failed to start capture");

      status = mmal_component_create("vc.null_sink", &null_sink_component);
      CHECK_STATUS(status, "failed to create null sink component");

      status = mmal_component_enable(null_sink_component);
      CHECK_STATUS(status, "failed to enable null sink component");

      status = mmal_connection_create(&preview_connection, camera->output[0], null_sink_component->input[0],
                                MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
      CHECK_STATUS(status, "failed to create preview connection");

      // The format of the two ports must have been committed before calling this function, although note that on creation, the connection automatically copies and commits the output port's format to the input port.
      status = mmal_connection_enable(preview_connection);
      CHECK_STATUS(status, "failed to enable preview connection");

      // output_port is enabled during drm_mmal_create_buffers
      if (!camera->output[0]->is_enabled)
      {
        status = mmal_port_enable(camera->output[0], output_callback);
        CHECK_STATUS(status, "failed to enable port 0");
      }

      printf("Connected preview port to null sink\n");
      // TODO: somewhere else, delete/destroy/release all these bits and pieces.
   }

   setup.out_fourcc = mmal_encoding_to_drm_fourcc(ENCODING_FOR_DRM);

   uint32_t con;
   ret = find_crtc(drmfd, &setup, &con);
   CHECK_CONDITION(ret, "failed to find valid mode\n");

   ret = find_plane(drmfd, &setup);
   CHECK_CONDITION(ret, "failed to find compatible plane\n");

   printf("Calling drm_mmal_craete_buffers with output_port = %i\n",output_port);
   status = drm_mmal_create_buffers(camera->output[output_port], buffers, drmfd, &pool_out);
   CHECK_STATUS(status, "failed to drm_mmal_create_buffers");

   /* Start decoding */
   fprintf(stderr, "start main loop\n");

   /* This is the main processing loop */
   while(!eos_received && out_count < 10000)
   {
      MMAL_BUFFER_HEADER_T *buffer;
      VCOS_STATUS_T vcos_status;

      /* Wait for buffer headers to be available on either of the camera ports */
      vcos_status = vcos_semaphore_wait_timeout(&context.semaphore, 2000);
      if (vcos_status != VCOS_SUCCESS)
         fprintf(stderr, "vcos_semaphore_wait_timeout failed - status %d\n", vcos_status);

      /* Get our output frames */
      while ((buffer = mmal_queue_get(context.queue)) != NULL)
      {
         /* We have a frame, do something with it (why not display it for instance?).
          * Once we're done with it, we release it. It will automatically go back
          * to its original pool so it can be reused for a new video frame.
          */
         eos_received = buffer->flags & MMAL_BUFFER_HEADER_FLAG_EOS;

         if (buffer->cmd)
         {
            fprintf(stderr, "received event %4.4s\n", (char *)&buffer->cmd);
            if (buffer->cmd == MMAL_EVENT_FORMAT_CHANGED)
            {
               fprintf(stderr, "Unexpected FORMAT_CHANGED event on the output of ISP\n");
            }
            mmal_buffer_header_release(buffer);
         }
         else
         {
            int index=0;

            fprintf(stderr, "decoded frame (flags %x) count %d\n", buffer->flags, out_count);
            if (buffer)
            {
               unsigned int i;
               for (i = 0; i < pool_out->headers_num; i++) {
                  if (buffers[i].mmal_buffer == buffer) {
                     //printf("Matches buffer index %u\n", i);
                     index = i;
                     break;
                  }
               }
               if (i == pool_out->headers_num) {
                  printf("Failed to find matching buffer for mmal buffer %p\n", buffer);
                  continue;
               }
            }
            else
               continue;

            ret = drmModeSetPlane(drmfd, setup.planeId, setup.crtcId,
                        buffers[index].fb_handle, 0,
                        setup.compose.x, setup.compose.y,
                        setup.compose.width,
                        setup.compose.height,
                        0, 0,
                        camera->output[output_port]->format->es->video.crop.width << 16,
                        camera->output[output_port]->format->es->video.crop.height << 16);
            CHECK_CONDITION(ret, "drmModeSetPlane failed: %s\n", ERRSTR);

            //Release buffer that was on the screen
            if (current_buffer)
               mmal_buffer_header_release(current_buffer);
            //Store pointer to the new buffer that is now on the screen
            current_buffer = buffer;
            out_count++;
         }
      }

      /* Send empty buffers to the output port of the camera */
      while ((buffer = mmal_queue_get(pool_out->queue)) != NULL)
      {
         //printf("Sending buf %p\n", buffer);
         status = mmal_port_send_buffer(camera->output[output_port], buffer);
         CHECK_STATUS(status, "failed to send buffer to camera");
      }
   }

   /* Stop decoding */
   fprintf(stderr, "stop decoding\n");

   /* Stop everything. Not strictly necessary since mmal_component_destroy()
    * will do that anyway */
   if (output_port != 0 && camera->output[0]->is_enabled)
   {
      mmal_port_disable(camera->output[0]);
   }
   mmal_component_disable(camera);

 error:
   /* Cleanup everything */
   if (preview_connection)
    {
        mmal_connection_disable(preview_connection);
        mmal_connection_destroy(preview_connection);
        preview_connection = NULL;
    }
   if (null_sink_component)
   {
        mmal_component_disable(null_sink_component);
        mmal_component_destroy(null_sink_component);
        null_sink_component = NULL;
   }

   status = drm_mmal_destroy_buffers(camera->output[output_port], buffers, drmfd, pool_out);
   if (pool_out)
      pool_destroy_drm(camera->output[output_port], pool_out, buffers, drmfd);
   if (camera)
      mmal_component_destroy(camera);
   if (context.queue)
      mmal_queue_destroy(context.queue);

   SOURCE_CLOSE();
   vcos_semaphore_delete(&context.semaphore);
   return status == MMAL_SUCCESS ? 0 : -1;
}
