/*
   Copyright (C) 2009 Red Hat, Inc.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are
   met:

       * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
       * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in
         the documentation and/or other materials provided with the
         distribution.
       * Neither the name of the copyright holder nor the names of its
         contributors may be used to endorse or promote products derived
         from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS
   IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
   TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
   PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _H_VD_AGENT
#define _H_VD_AGENT

#include <spice/types.h>

#include <spice/start-packed.h>

enum {
    VDP_CLIENT_PORT = 1,
    VDP_SERVER_PORT,
    VDP_END_PORT
};

typedef struct SPICE_ATTR_PACKED VDIChunkHeader {
    uint32_t port;
    uint32_t size;
} VDIChunkHeader;

typedef struct SPICE_ATTR_PACKED VDAgentMessage {
    /* Should be VD_AGENT_PROTOCOL */
    uint32_t protocol;
    /* One of VD_AGENT_xxx in the enumeration below */
    uint32_t type;
    uint64_t opaque;
    /* Size of data following */
    uint32_t size;
    uint8_t data[0];
} VDAgentMessage;

#define VD_AGENT_PROTOCOL 1
#define VD_AGENT_MAX_DATA_SIZE 2048

/*
 * Messages and types for guest agent.
 * These messages are sent through the virtio port "com.redhat.spice.0"
 * (agent <-> server) or embedded in "agent_data" SPICE protocol message in
 * the "MainChannel" (server <-> client)
 */
enum {
    /* server -> agent
     * See VDAgentMouseState structure.
     */
    VD_AGENT_MOUSE_STATE = 1,
    /* client -> agent|server.
     * Acknowledged by the agent using VD_AGENT_REPLY.
     * See VDAgentMonitorsConfig structure.
     */
    VD_AGENT_MONITORS_CONFIG,
    /* agent -> client.
     * See VDAgentReply structure.
     */
    VD_AGENT_REPLY,
    /* Set clipboard data (both directions).
     * Message comes with type and data.
     * See VDAgentClipboard structure.
     */
    VD_AGENT_CLIPBOARD,
    /* client -> agent.
     * Acknowledged by Windows agent using VD_AGENT_REPLY.
     * See VDAgentDisplayConfig structure.
    */
    VD_AGENT_DISPLAY_CONFIG,
    /* See VDAgentAnnounceCapabilities structure. */
    VD_AGENT_ANNOUNCE_CAPABILITIES,
    /* Asks to listen for clipboard changes (both directions).
     * Remote should empty clipboard and wait for one
     * of the types passed.
     * See VDAgentClipboardGrab structure.
     */
    VD_AGENT_CLIPBOARD_GRAB,
    /* Asks for clipboard data (both directions).
     * Request comes with a specific type.
     * See VDAgentClipboardRequest structure.
     */
    VD_AGENT_CLIPBOARD_REQUEST,
    /* See VDAgentClipboardRelease structure. */
    VD_AGENT_CLIPBOARD_RELEASE,
    /* See VDAgentFileXferStartMessage structure. */
    VD_AGENT_FILE_XFER_START,
    /* See VDAgentFileXferStatusMessage structure. */
    VD_AGENT_FILE_XFER_STATUS,
    /* See VDAgentFileXferDataMessage structure. */
    VD_AGENT_FILE_XFER_DATA,
    /* Empty message */
    VD_AGENT_CLIENT_DISCONNECTED,
    /* See VDAgentMaxClipboard structure. */
    VD_AGENT_MAX_CLIPBOARD,
    /* See VDAgentAudioVolumeSync structure. */
    VD_AGENT_AUDIO_VOLUME_SYNC,
    /* See VDAgentGraphicsDeviceInfo structure. */
    VD_AGENT_GRAPHICS_DEVICE_INFO,
    VD_AGENT_END_MESSAGE,
};

enum {
    VD_AGENT_FILE_XFER_STATUS_CAN_SEND_DATA,
    VD_AGENT_FILE_XFER_STATUS_CANCELLED,
    VD_AGENT_FILE_XFER_STATUS_ERROR,
    VD_AGENT_FILE_XFER_STATUS_SUCCESS,
    VD_AGENT_FILE_XFER_STATUS_NOT_ENOUGH_SPACE,
    VD_AGENT_FILE_XFER_STATUS_SESSION_LOCKED,
    VD_AGENT_FILE_XFER_STATUS_VDAGENT_NOT_CONNECTED,
    VD_AGENT_FILE_XFER_STATUS_DISABLED,
};

typedef struct SPICE_ATTR_PACKED VDAgentFileXferStatusMessage {
    uint32_t id;
    uint32_t result;
    /* Used to send additional data for detailed error messages
     * to clients with VD_AGENT_CAP_FILE_XFER_DETAILED_ERRORS capability.
     * Type of data varies with the result:
     * result : data type (NULL if no data)
     * VD_AGENT_FILE_XFER_STATUS_ERROR : VDAgentFileXferStatusError
     * VD_AGENT_FILE_XFER_STATUS_NOT_ENOUGH_SPACE : VDAgentFileXferStatusNotEnoughSpace
     * VD_AGENT_FILE_XFER_STATUS_SESSION_LOCKED : NULL
     * VD_AGENT_FILE_XFER_STATUS_VDAGENT_NOT_CONNECTED : NULL
     * VD_AGENT_FILE_XFER_STATUS_DISABLED : NULL
     */
    uint8_t data[0];
} VDAgentFileXferStatusMessage;

/* Detailed error for VD_AGENT_FILE_XFER_STATUS_NOT_ENOUGH_SPACE.
 * Only present if VD_AGENT_CAP_FILE_XFER_DETAILED_ERRORS is
 * negotiated and the size of the message can contain it.
 */
typedef struct SPICE_ATTR_PACKED VDAgentFileXferStatusNotEnoughSpace {
    /* Disk free space in bytes. */
    uint64_t disk_free_space;
} VDAgentFileXferStatusNotEnoughSpace;

enum {
    /* Error number is a G_IO_ERROR_xxx defined in
     * https://developer.gnome.org/gio/stable/gio-GIOError.html
     */
    VD_AGENT_FILE_XFER_STATUS_ERROR_GLIB_IO,
};

/* Detailed error for VD_AGENT_FILE_XFER_STATUS_ERROR.
 * Only present if VD_AGENT_CAP_FILE_XFER_DETAILED_ERRORS is
 * negotiated and the size of the message can contain it.
 * Otherwise a generic error should be assumed and reported.
 */
typedef struct SPICE_ATTR_PACKED VDAgentFileXferStatusError {
    /* One of VD_AGENT_FILE_XFER_STATUS_ERROR_xxx enumeration
     */
    uint8_t error_type;
    /* An error code which enumeration depends on error_type
     */
    uint32_t error_code;
} VDAgentFileXferStatusError;

typedef struct SPICE_ATTR_PACKED VDAgentFileXferStartMessage {
    uint32_t id;
    uint8_t data[0];
} VDAgentFileXferStartMessage;

typedef struct SPICE_ATTR_PACKED VDAgentFileXferDataMessage {
    uint32_t id;
    uint64_t size;
    uint8_t data[0];
} VDAgentFileXferDataMessage;

typedef struct SPICE_ATTR_PACKED VDAgentMonConfig {
    /*
     * Note a width and height of 0 can be used to indicate a disabled
     * monitor, this may only be used with agents with the
     * VD_AGENT_CAP_SPARSE_MONITORS_CONFIG capability.
     */
    uint32_t height;
    uint32_t width;
    uint32_t depth;
    int32_t x;
    int32_t y;
} VDAgentMonConfig;

enum {
    VD_AGENT_CONFIG_MONITORS_FLAG_USE_POS = (1 << 0),
    VD_AGENT_CONFIG_MONITORS_FLAG_PHYSICAL_SIZE = (1 << 1),
};

typedef struct SPICE_ATTR_PACKED VDAgentMonitorsConfig {
    uint32_t num_of_monitors;
    uint32_t flags;
    VDAgentMonConfig monitors[0];
    /* only sent if the FLAG_PHYSICAL_SIZE is present: */
    /* VDAgentMonitorMM physical_sizes[0]; */
} VDAgentMonitorsConfig;


/* Physical size of the monitor in millimeters.
 * Having this information, the remote/guest display can configure itself with
 * appropriate font & scaling to maintain readability. */
typedef struct SPICE_ATTR_PACKED VDAgentMonitorMM {
    /*
     * Note a width and height of 0 can be used to indicate a disabled
     * monitor or no size information is present.
     */
    uint16_t height;
    uint16_t width;
} VDAgentMonitorMM;

enum {
    VD_AGENT_DISPLAY_CONFIG_FLAG_DISABLE_WALLPAPER = (1 << 0),
    VD_AGENT_DISPLAY_CONFIG_FLAG_DISABLE_FONT_SMOOTH = (1 << 1),
    VD_AGENT_DISPLAY_CONFIG_FLAG_DISABLE_ANIMATION = (1 << 2),
    VD_AGENT_DISPLAY_CONFIG_FLAG_SET_COLOR_DEPTH = (1 << 3),
};

typedef struct SPICE_ATTR_PACKED VDAgentDisplayConfig {
    uint32_t flags;
    uint32_t depth;
} VDAgentDisplayConfig;

#define VD_AGENT_LBUTTON_MASK (1 << 1)
#define VD_AGENT_MBUTTON_MASK (1 << 2)
#define VD_AGENT_RBUTTON_MASK (1 << 3)
#define VD_AGENT_UBUTTON_MASK (1 << 4)
#define VD_AGENT_DBUTTON_MASK (1 << 5)
#define VD_AGENT_SBUTTON_MASK (1 << 6)
#define VD_AGENT_EBUTTON_MASK (1 << 7)

typedef struct SPICE_ATTR_PACKED VDAgentMouseState {
    uint32_t x;
    uint32_t y;
    uint32_t buttons;
    uint8_t display_id;
} VDAgentMouseState;

typedef struct SPICE_ATTR_PACKED VDAgentReply {
    uint32_t type;
    uint32_t error;
} VDAgentReply;

enum {
    VD_AGENT_SUCCESS = 1,
    VD_AGENT_ERROR,
};

typedef struct SPICE_ATTR_PACKED VDAgentClipboard {
#if 0 /* VD_AGENT_CAP_CLIPBOARD_SELECTION */
    uint8_t selection;
    uint8_t __reserved[sizeof(uint32_t) - 1 * sizeof(uint8_t)];
#endif
    uint32_t type;
    uint8_t data[0];
} VDAgentClipboard;

enum {
    VD_AGENT_CLIPBOARD_NONE = 0,
    VD_AGENT_CLIPBOARD_UTF8_TEXT,
    VD_AGENT_CLIPBOARD_IMAGE_PNG,  /* All clients with image support should support this one */
    VD_AGENT_CLIPBOARD_IMAGE_BMP,  /* optional */
    VD_AGENT_CLIPBOARD_IMAGE_TIFF, /* optional */
    VD_AGENT_CLIPBOARD_IMAGE_JPG,  /* optional */
    /* identifies a list of absolute paths in phodav server
     * that is associated with the "org.spice-space.webdav.0" webdav channel;
     * the items are encoded in UTF-8 and separated by '\0';
     * the first item must be either "copy" or "cut" (without the quotes)
     * to indicate what action should be performed with the files that follow */
    VD_AGENT_CLIPBOARD_FILE_LIST,
};

enum {
    VD_AGENT_CLIPBOARD_SELECTION_CLIPBOARD = 0,
    VD_AGENT_CLIPBOARD_SELECTION_PRIMARY,
    VD_AGENT_CLIPBOARD_SELECTION_SECONDARY,
};

typedef struct SPICE_ATTR_PACKED VDAgentClipboardGrab {
#if 0 /* VD_AGENT_CAP_CLIPBOARD_SELECTION */
    uint8_t selection;
    uint8_t __reserved[sizeof(uint32_t) - 1 * sizeof(uint8_t)];
#endif
#if 0 /* VD_AGENT_CAP_CLIPBOARD_GRAB_SERIAL */
    uint32_t serial;
#endif
    uint32_t types[0];
} VDAgentClipboardGrab;

typedef struct SPICE_ATTR_PACKED VDAgentClipboardRequest {
#if 0 /* VD_AGENT_CAP_CLIPBOARD_SELECTION */
    uint8_t selection;
    uint8_t __reserved[sizeof(uint32_t) - 1 * sizeof(uint8_t)];
#endif
    uint32_t type;
} VDAgentClipboardRequest;

typedef struct SPICE_ATTR_PACKED VDAgentClipboardRelease {
#if 0 /* VD_AGENT_CAP_CLIPBOARD_SELECTION */
    uint8_t selection;
    uint8_t __reserved[sizeof(uint32_t) - 1 * sizeof(uint8_t)];
#endif
    uint8_t dummy_empty_field[0]; /* C/C++ compatibility */
} VDAgentClipboardRelease;

typedef struct SPICE_ATTR_PACKED VDAgentMaxClipboard {
    int32_t max;
} VDAgentMaxClipboard;

typedef struct SPICE_ATTR_PACKED VDAgentAudioVolumeSync {
    uint8_t is_playback;
    uint8_t mute;
    uint8_t nchannels;
    uint16_t volume[0];
} VDAgentAudioVolumeSync;

typedef struct SPICE_ATTR_PACKED VDAgentDeviceDisplayInfo {
    uint32_t channel_id;
    uint32_t monitor_id;
    uint32_t device_display_id;
    uint32_t device_address_len;
    uint8_t device_address[0];  /* a zero-terminated string */
} VDAgentDeviceDisplayInfo;


/* This message contains the mapping of (channel_id, monitor_id) pair to a
 * "physical" (virtualized) device and its monitor identified by device_address
 * and device_display_id.
 *
 * It's used on the vd_agent to identify the guest monitors for the
 * mouse_position and monitors_config messages.
 */
typedef struct SPICE_ATTR_PACKED VDAgentGraphicsDeviceInfo {
    uint32_t count;
#ifdef _MSC_VER
    uint8_t display_info[0];
#else
    VDAgentDeviceDisplayInfo display_info[0];
#endif
} VDAgentGraphicsDeviceInfo;


/* Capabilities definitions
 */
enum {
    VD_AGENT_CAP_MOUSE_STATE = 0,
    VD_AGENT_CAP_MONITORS_CONFIG,
    VD_AGENT_CAP_REPLY,
    VD_AGENT_CAP_CLIPBOARD,
    VD_AGENT_CAP_DISPLAY_CONFIG,
    VD_AGENT_CAP_CLIPBOARD_BY_DEMAND,
    VD_AGENT_CAP_CLIPBOARD_SELECTION,
    VD_AGENT_CAP_SPARSE_MONITORS_CONFIG,
    VD_AGENT_CAP_GUEST_LINEEND_LF,
    VD_AGENT_CAP_GUEST_LINEEND_CRLF,
    VD_AGENT_CAP_MAX_CLIPBOARD,
    VD_AGENT_CAP_AUDIO_VOLUME_SYNC,
    VD_AGENT_CAP_MONITORS_CONFIG_POSITION,
    VD_AGENT_CAP_FILE_XFER_DISABLED,
    VD_AGENT_CAP_FILE_XFER_DETAILED_ERRORS,
    VD_AGENT_CAP_GRAPHICS_DEVICE_INFO,
    VD_AGENT_CAP_CLIPBOARD_NO_RELEASE_ON_REGRAB,
    VD_AGENT_CAP_CLIPBOARD_GRAB_SERIAL,
    VD_AGENT_END_CAP,
};

typedef struct SPICE_ATTR_PACKED VDAgentAnnounceCapabilities {
    uint32_t  request;
    uint32_t caps[0];
} VDAgentAnnounceCapabilities;

#define VD_AGENT_CAPS_SIZE_FROM_MSG_SIZE(msg_size) \
    (((msg_size) - sizeof(VDAgentAnnounceCapabilities)) / sizeof(uint32_t))

#define VD_AGENT_CAPS_SIZE ((VD_AGENT_END_CAP + 31) / 32)

#define VD_AGENT_CAPS_BYTES (((VD_AGENT_END_CAP + 31) / 8) & ~3)

#define VD_AGENT_HAS_CAPABILITY(caps, caps_size, index) \
    ((index) < (caps_size * 32) && ((caps)[(index) / 32] & (1 << ((index) % 32))))

#define VD_AGENT_SET_CAPABILITY(caps, index) \
    { (caps)[(index) / 32] |= (1 << ((index) % 32)); }

#define VD_AGENT_CLEAR_CAPABILITY(caps, index) \
    { (caps)[(index) / 32] &= ~(1 << ((index) % 32)); }

#include <spice/end-packed.h>

#endif /* _H_VD_AGENT */
