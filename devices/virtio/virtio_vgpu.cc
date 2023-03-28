/*
 * MVisor VirtIO-VGPU Device
 * It delivers virgl3d commands from guest to host libvirglrenderer.so,
 * but it doesn't display anything for guest vm.
 * Copyright (C) 2022 cair <rui.cai@tenclass.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "logger.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <filesystem>
#include "virtio_pci.h"
#include "virtio_vgpu.h"
#include "machine.h"
#include "memory_manager.h"
#include "device_manager.h"
#include "virtio_vgpu.pb.h"

#ifdef __cplusplus
extern "C" {
  #include <virglrenderer.h>
}
#endif

struct VgpuCommand {
  uint64_t      fence_id;
  VirtQueue     &vq;
  VirtElement   *element;
};

struct VirglResource {
  uint64_t  gpa;
  size_t    size;
  struct virgl_renderer_resource_create_args args;
};

static struct virgl_renderer_callbacks vgpu_cbs = {
  .version      = 2,
  .write_fence  = virgl_write_fence,
  .get_drm_fd = virgl_get_drm_fd
};

class VirtioVgpu : public VirtioPci {
 private:
  struct vgpu_config vgpu_config_;
  IoTimer* fence_poll_timer_;
  bool initialized_;
  bool reset_;

  std::list<VgpuCommand> commands_;
  // <virgl_context_id, <virgl_resource_id, virgl_resource>>
  std::map<uint32_t, std::map<uint32_t, VirglResource>> virgl_resources_;
  // <virgl_context_id, virgl_cmd>
  std::map<uint32_t, std::vector<std::string>> virgl_cmds_;

  // base address for blob resource mapping
  uint64_t blob_memory_gpa_base_;
  // <virgl_context_id, <virgl_resource_id, MemoryRegion*>>
  std::map<uint32_t, std::map<uint32_t, const MemoryRegion*>> blob_memory_regions_;
  
 public:
  VirtioVgpu() {
    pci_header_.vendor_id = 0x1AF4;
    pci_header_.subsys_id = 0x1100;
    pci_header_.device_id = 0x105B;
    pci_header_.revision_id = 0x01;
    device_features_ |= 0;

    reset_ = false;
    initialized_ = false;

    // blob resource memory region base address
    blob_memory_gpa_base_ = 256LL << 30;

    // only support 2 queues now, one for command and another one for control
    vgpu_config_.num_queues = 2;
    // vgpu memory size must be smaller than system ram size
    vgpu_config_.memory_size = 1LL << 30;
    vgpu_config_.capabilities = VIRTGPU_PARAM_3D_FEATURES |
                                VIRTGPU_PARAM_CAPSET_QUERY_FIX |
                                VIRTGPU_PARAM_CONTEXT_INIT |
                                VIRTGPU_PARAM_SUPPORTED_CAPSET_IDs;
    // use staging to control the type of resource memory allocation
    vgpu_config_.staging = false;

    // one msi item for one queue
    MV_ASSERT(vgpu_config_.num_queues <= 4);

    AddPciBar(1, 0x1000, kIoResourceTypeMmio);
    AddMsiXCapability(1, 4, 0, 0x1000);
  }

  virtual void Disconnect() {
    VirtioPci::Disconnect();

    if (fence_poll_timer_) {
      RemoveTimer(fence_poll_timer_);
      fence_poll_timer_ = nullptr;
    }
    
    // FIXME: virgl cleanup may cause crash when the guest didn't destroy all virgl contexts in AMD gpu
    virgl_renderer_cleanup(this);
  }

  virtual void Connect() {
    if (has_key("memory")) { 
      std::string memory = std::get<std::string>(key_values_["memory"]);
      if (memory.back() != 'G') {
        MV_PANIC("memory size must be aligned with 1GB");
      }
      auto value = atol(memory.substr(0, memory.length() - 1).c_str());
      vgpu_config_.memory_size = (1UL << 30) * value;
    }

    if (has_key("blob") && std::get<bool>(key_values_["blob"])) {
      // add experimental feature blob support
      vgpu_config_.capabilities |= VIRTGPU_PARAM_RESOURCE_BLOB | VIRTGPU_PARAM_HOST_VISIBLE;
    }

    if (has_key("staging")) {
      vgpu_config_.staging = std::get<bool>(key_values_["staging"]);
    }

    // get num_capsets from virglrenderer
    uint32_t capset2_max_ver = 0, capset2_max_size = 0;
    virgl_renderer_get_cap_set(VIRTIO_GPU_CAPSET_VIRGL2, &capset2_max_ver, &capset2_max_size);
    vgpu_config_.num_capsets = capset2_max_ver > 0 ? 2 : 1;

    // check fences and call write_fence callback
    fence_poll_timer_ = AddTimer(1, true, [this]() {
      if (!commands_.empty()) {
        virgl_renderer_poll();
      }
    });

    VirtioPci::Connect();
  }

  void WriteFence(uint32_t fence_id) {
    for (auto it = commands_.begin(); it != commands_.end();) {
      if (it->fence_id > fence_id) {
        it++;
      } else {
        PushQueue(it->vq, it->element);
        NotifyQueue(it->vq);
        it = commands_.erase(it);
      }
    }
  }

  int GetDrmFd() {
    std::string node;
    if (has_key("node")) { 
      node = std::get<std::string>(key_values_["node"]);
    } else {
      std::vector<std::string> render_nodes;
      for (auto& entry : std::filesystem::directory_iterator("/dev/dri")) {
        if (entry.is_directory()) {
          continue;
        }

        auto filename = entry.path().filename().string();
        if (filename.length() > 8 && filename.substr(0, 7) == "renderD") {
          render_nodes.push_back(entry.path().string());
        }
      }
      MV_ASSERT(render_nodes.size() > 0);
      node = render_nodes[getpid() % render_nodes.size()];
    }

    auto fd = open(node.c_str(), O_RDWR | O_CLOEXEC | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      MV_PANIC("open render node=%s failed", node.c_str());
    }
    return fd;
  }

  void Reset() {
    VirtioPci::Reset();

    for (uint32_t i = 0; i < vgpu_config_.num_queues; ++i) {
      // mesa->virglrederer need a large queue to produce/consume commands 
      AddQueue(1024 * 32, std::bind(&VirtioVgpu::OnOutput, this, i));
    }
    reset_ = true;
  }

  void ReadDeviceConfig(uint64_t offset, uint8_t* data, uint32_t size) {
    MV_ASSERT(offset + size <= sizeof(vgpu_config_));
    memcpy(data, (uint8_t*)&vgpu_config_ + offset, size);
  }

  void OnOutput(int queue_index) {
    bool notify = false;
    auto& vq = queues_[queue_index];

    while (auto element = PopQueue(vq)) {
      auto fence_id = HandleCommand(element);
      if (fence_id == 0) {
        PushQueue(vq, element);
        notify = true;
      } else {
        commands_.emplace_back(VgpuCommand {
          .fence_id = fence_id,
          .vq = vq,
          .element = element
        });
      }
    }

    if (notify) {
      NotifyQueue(vq);
    }
  }

  void HandleVirglCommand(void* buf, size_t size, VirglCommandCallback callback) {
    uint32_t index = 0;
    uint32_t *cmd_buf = (uint32_t*)buf;
    uint32_t cmd_count = size / sizeof(uint32_t);
    while (index < cmd_count) {
      uint32_t *cmd = &cmd_buf[index];
      uint32_t len = cmd[0] >> 16;
      callback(cmd);
      index += len + 1;
    }
  }

  inline bool VirglResourceExist(uint32_t context_id, uint32_t res_handle) {
    if (virgl_resources_[context_id].find(res_handle) != virgl_resources_[context_id].end()) {
      return true;
    }
    return false;
  }

  bool SaveState(MigrationWriter* writer) {
    // no command left
    MV_ASSERT(commands_.empty());

    if (vgpu_config_.capabilities & (VIRTGPU_PARAM_RESOURCE_BLOB | VIRTGPU_PARAM_HOST_VISIBLE)) {
      MV_PANIC("host blob resource was not supported in migration");
    }

    if (vgpu_config_.capabilities & VIRTGPU_PARAM_CAPSET_QUERY_FIX) {
      // migrate guest with staging=true may cause resource data loss
      MV_ASSERT(!vgpu_config_.staging);
    }

    VirtioVgpuState state;
    size_t valid_cmd_size = 0, total_cmd_size = 0;
    for (auto iter = virgl_resources_.begin(); iter != virgl_resources_.end(); iter++) {
      auto& context_id = iter->first;
      auto& virgl_resource_map = iter->second;

      // save virgl context id
      auto context = state.add_virgl_contexts();
      context->set_context_id(context_id);

      // save virgl resource for current virgl context
      for (auto iter1 = virgl_resource_map.begin(); iter1 != virgl_resource_map.end(); iter1++) {
        auto& resource = iter1->second;
        auto item = context->add_virgl_resources();
        item->set_gpa(resource.gpa);
        item->set_size(resource.size);
        item->set_resource_args(&resource.args, sizeof(resource.args));
      }

      // save virgl cmds for current virgl context
      // iterate through cmds to get invalid information first
      std::unordered_set<uint32_t> invalid_object_handles;
      std::unordered_set<uint32_t> invalid_sub_context_ids;
      for (auto iter1 = virgl_cmds_[context_id].begin(); iter1 != virgl_cmds_[context_id].end(); iter1++) {
        HandleVirglCommand(iter1->data(), iter1->size(), [&](uint32_t* cmd) {
          uint32_t cmd_index = cmd[0] & 0xff;
          switch (cmd_index) {
            case VIRGL_CCMD_DESTROY_OBJECT:
              invalid_object_handles.insert(cmd[VIRGL_OBJ_DESTROY_HANDLE]);
              break;
            case VIRGL_CCMD_DESTROY_SUB_CTX:
              invalid_sub_context_ids.insert(cmd[1]);
            default:
              break;
          }
        });
      }

      // iterate through cmds to get valid cmds second
      for (auto iter1 = virgl_cmds_[context_id].begin(); iter1 != virgl_cmds_[context_id].end();) {
        std::string valid_cmds_str;
        total_cmd_size += iter1->size();

        // iterate
        HandleVirglCommand(iter1->data(), iter1->size(), [&, this](uint32_t* cmd) {
          bool valid = true;
          uint32_t length = cmd[0] >> 16;
          uint32_t cmd_index = cmd[0] & 0xff;
          switch (cmd_index) {
            case VIRGL_CCMD_CREATE_OBJECT: {
              uint32_t create_handle = cmd[VIRGL_OBJ_CREATE_HANDLE];
              if (invalid_object_handles.find(create_handle) != invalid_object_handles.end()) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_BIND_OBJECT: {
              uint32_t bind_handle = cmd[VIRGL_OBJ_BIND_HANDLE];
              if (invalid_object_handles.find(bind_handle) != invalid_object_handles.end()) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_SET_SAMPLER_VIEWS: {
              uint32_t invalid_num = 0;
              uint32_t sampler_views_num = length - 2;
              for (size_t i = 0; i < sampler_views_num; i++) {
                if (invalid_object_handles.find(cmd[VIRGL_SET_SAMPLER_VIEWS_V0_HANDLE + i]) != invalid_object_handles.end()) {
                  cmd[VIRGL_SET_SAMPLER_VIEWS_V0_HANDLE + i] = 0;
                  invalid_num++;
                }
              }
              if (invalid_num == sampler_views_num) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_SET_FRAMEBUFFER_STATE: {
              uint32_t zsurface_handle = cmd[VIRGL_SET_FRAMEBUFFER_STATE_NR_ZSURF_HANDLE];
              if (zsurface_handle > 0 && invalid_object_handles.find(zsurface_handle) != invalid_object_handles.end()) {
                valid = false;
                break;
              }

              uint32_t invalid_count = 0;
              uint32_t surface_handle_num = cmd[VIRGL_SET_FRAMEBUFFER_STATE_NR_CBUFS];
              for (size_t i = 0; i < surface_handle_num; i++) {
                uint32_t surface_handle = cmd[VIRGL_SET_FRAMEBUFFER_STATE_CBUF_HANDLE(i)];
                if (surface_handle > 0 && invalid_object_handles.find(surface_handle) != invalid_object_handles.end()) {
                  invalid_count++;
                }
              }
              if (invalid_count == surface_handle_num) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_TRANSFER3D: {
              uint32_t res_handle = cmd[VIRGL_RESOURCE_IW_RES_HANDLE];
              if (!VirglResourceExist(context_id, res_handle)) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_BLIT: {
              uint32_t src_res_handle = cmd[VIRGL_CMD_BLIT_SRC_RES_HANDLE];
              if (!VirglResourceExist(context_id, src_res_handle)) {
                valid = false;
                break;
              }

              uint32_t dst_res_handle = cmd[VIRGL_CMD_BLIT_DST_RES_HANDLE];
              if (!VirglResourceExist(context_id, dst_res_handle)) {
                valid = false;
                break;
              }
              break;
            }
            case VIRGL_CCMD_BEGIN_QUERY: {
              uint32_t begin_query_handle = cmd[VIRGL_QUERY_BEGIN_HANDLE];
              if (invalid_object_handles.find(begin_query_handle) != invalid_object_handles.end()) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_END_QUERY: {
              uint32_t end_query_handle = cmd[VIRGL_QUERY_END_HANDLE];
              if (invalid_object_handles.find(end_query_handle) != invalid_object_handles.end()) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_BIND_SHADER: {
              uint32_t bind_shader_handle = cmd[VIRGL_BIND_SHADER_HANDLE];
              if (invalid_object_handles.find(bind_shader_handle) != invalid_object_handles.end()) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_SET_SHADER_BUFFERS: {
              uint32_t invalid_count = 0;
              uint32_t shader_buffers_num = (length - 2) / VIRGL_SET_SHADER_BUFFER_ELEMENT_SIZE;
              for (size_t i = 0; i < shader_buffers_num; i++) {
                uint32_t res_handle = cmd[VIRGL_SET_SHADER_BUFFER_RES_HANDLE(i)];
                if (!VirglResourceExist(context_id, res_handle)) {
                  invalid_count++;
                }
              }
              if (invalid_count == shader_buffers_num) {
                 valid = false;
              }
              break;
            }
            case VIRGL_CCMD_SET_SHADER_IMAGES: {
              uint32_t invalid_count = 0;
              uint32_t shader_images_num = (length - 2) / VIRGL_SET_SHADER_IMAGE_ELEMENT_SIZE;
              for (size_t i = 0; i < shader_images_num; i++) {
                uint32_t res_handle = cmd[VIRGL_SET_SHADER_IMAGE_RES_HANDLE(i)];
                if (!VirglResourceExist(context_id, res_handle)) {
                  invalid_count++;
                }
              }
              if (invalid_count == shader_images_num) {
                 valid = false;
              }
              break;
            }
            case VIRGL_CCMD_LINK_SHADER: {
              uint32_t vertex_shader_handle = cmd[VIRGL_LINK_SHADER_VERTEX_HANDLE];
              if (vertex_shader_handle > 0 && invalid_object_handles.find(vertex_shader_handle) != invalid_object_handles.end()) {
                valid = false;
                break;
              }
              uint32_t fragment_shader_handle = cmd[VIRGL_LINK_SHADER_FRAGMENT_HANDLE];
              if (fragment_shader_handle > 0 && invalid_object_handles.find(fragment_shader_handle) != invalid_object_handles.end()) {
                valid = false;
                break;
              }
              uint32_t geometry_shader_handle = cmd[VIRGL_LINK_SHADER_GEOMETRY_HANDLE];
              if (geometry_shader_handle > 0 && invalid_object_handles.find(geometry_shader_handle) != invalid_object_handles.end()) {
                valid = false;
                break;
              }
              uint32_t tess_ctrl_shader_handle = cmd[VIRGL_LINK_SHADER_TESS_CTRL_HANDLE];
              if (tess_ctrl_shader_handle > 0 && invalid_object_handles.find(tess_ctrl_shader_handle) != invalid_object_handles.end()) {
                valid = false;
                break;
              }
              uint32_t tess_eval_shader_handle = cmd[VIRGL_LINK_SHADER_TESS_EVAL_HANDLE];
              if (tess_eval_shader_handle > 0 && invalid_object_handles.find(tess_eval_shader_handle) != invalid_object_handles.end()) {
                valid = false;
                break;
              }
              uint32_t compute_shader_handle = cmd[VIRGL_LINK_SHADER_COMPUTE_HANDLE];
              if (compute_shader_handle > 0 && invalid_object_handles.find(compute_shader_handle) != invalid_object_handles.end()) {
                valid = false;
                break;
              } 
              break;
            }
            case VIRGL_CCMD_COPY_TRANSFER3D: {
              MV_PANIC("VIRGL_CCMD_COPY_TRANSFER3D was not supported in migration");
              break;
            }
            case VIRGL_CCMD_SET_RENDER_CONDITION: {
              uint32_t render_condition_handle = cmd[VIRGL_RENDER_CONDITION_HANDLE];
              if (render_condition_handle > 0 && invalid_object_handles.find(render_condition_handle) != invalid_object_handles.end()) {
                valid = false;
                break;
              } 
              break;
            }
            case VIRGL_CCMD_SET_STREAMOUT_TARGETS: {
              uint32_t targets_num = length - 1;
              if (targets_num == 0) {
                break;
              }
              uint32_t invalid_count = 0;
              for (size_t i = 0; i < targets_num; i++) {
                uint32_t target_handle = cmd[VIRGL_SET_STREAMOUT_TARGETS_H0 + i];
                if (invalid_object_handles.find(target_handle) != invalid_object_handles.end()) {
                  invalid_count++;
                }
              }
              if (invalid_count == targets_num) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_RESOURCE_COPY_REGION: {
              uint32_t src_res_handle = cmd[VIRGL_CMD_RCR_SRC_RES_HANDLE];
              if (!VirglResourceExist(context_id, src_res_handle)) {
                valid = false;
                break;
              }

              uint32_t dst_res_handle = cmd[VIRGL_CMD_RCR_DST_RES_HANDLE];
              if (!VirglResourceExist(context_id, dst_res_handle)) {
                valid = false;
                break;
              }
              break;
            }
            case VIRGL_CCMD_BIND_SAMPLER_STATES: {
              uint32_t invalid_count = 0;
              uint32_t sampler_states_num = length - 2;
              uint32_t* handles = &cmd[VIRGL_BIND_SAMPLER_STATES_S0_HANDLE];
              for (size_t i = 0; i < sampler_states_num; i++) {
                if (handles[i] > 0 && invalid_object_handles.find(handles[i]) != invalid_object_handles.end()) {
                  invalid_count++;
                }
              }
              if (invalid_count == sampler_states_num) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_SET_INDEX_BUFFER: {
              uint32_t index_buffer_handle = cmd[VIRGL_SET_INDEX_BUFFER_HANDLE];
              if (index_buffer_handle > 0 && !VirglResourceExist(context_id, index_buffer_handle)) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_RESOURCE_INLINE_WRITE: {
              uint32_t dst_res_handle = cmd[VIRGL_RESOURCE_IW_RES_HANDLE];
              if (!VirglResourceExist(context_id, dst_res_handle)) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_SET_UNIFORM_BUFFER: {
              uint32_t res_handle = cmd[VIRGL_SET_UNIFORM_BUFFER_RES_HANDLE];
              if (!VirglResourceExist(context_id, res_handle)) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_SET_VERTEX_BUFFERS: {
              uint32_t invalid_count = 0;
              uint32_t vertex_buffers_num = length / 3;
              for (size_t i = 0; i < vertex_buffers_num; i++) {
                uint32_t res_handle = cmd[VIRGL_SET_VERTEX_BUFFER_HANDLE(i)];
                if (res_handle > 0 && !VirglResourceExist(context_id, res_handle)) {
                  invalid_count++;
                }
              }
              if (invalid_count == vertex_buffers_num) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_DRAW_VBO: {
              if (length != VIRGL_DRAW_VBO_SIZE_INDIRECT) {
                break;
              }
              uint32_t vbo_indirect_handle = cmd[VIRGL_DRAW_VBO_INDIRECT_HANDLE];
              if (vbo_indirect_handle > 0 && !VirglResourceExist(context_id, vbo_indirect_handle)) {
                valid = false;
                break;
              }
              uint32_t vbo_indirect_draw_count_handle = cmd[VIRGL_DRAW_VBO_INDIRECT_DRAW_COUNT_HANDLE];
              if (vbo_indirect_draw_count_handle > 0 && !VirglResourceExist(context_id, vbo_indirect_draw_count_handle)) {
                valid = false;
                break;
              }
              break;
            }
            case VIRGL_CCMD_CLEAR_TEXTURE: {
              uint32_t texture_handle = cmd[VIRGL_TEXTURE_HANDLE];
              if (!VirglResourceExist(context_id, texture_handle)) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_SET_SUB_CTX:
            case VIRGL_CCMD_CREATE_SUB_CTX: {
              uint32_t sub_context_id = cmd[1];
              if (invalid_sub_context_ids.find(sub_context_id) != invalid_sub_context_ids.end()) {
                valid = false;
              }
              break;
            }
            case VIRGL_CCMD_PIPE_RESOURCE_CREATE:
            case VIRGL_CCMD_DESTROY_OBJECT:
            case VIRGL_CCMD_DESTROY_SUB_CTX:
            case VIRGL_CCMD_GET_MEMORY_INFO:
            case VIRGL_CCMD_GET_QUERY_RESULT:
            case VIRGL_CCMD_GET_QUERY_RESULT_QBO:
            case VIRGL_CCMD_END_TRANSFERS:
            case VIRGL_CCMD_NOP:
              valid = false;
              break;
            default:
              break;
          }

          if (valid) {
            std::string cmd_str((char*)cmd, (length + 1) * sizeof(uint32_t));
            valid_cmds_str += cmd_str;
          }
        });

        // save valid virgl cmds
        if (valid_cmds_str.empty()) {
          iter1 = virgl_cmds_[context_id].erase(iter1);
        } else {
          context->add_virgl_cmds(valid_cmds_str.data(), valid_cmds_str.size());
          valid_cmd_size += valid_cmds_str.size();

          // update virgl cmd buffer saved
          *iter1 = std::move(valid_cmds_str);
          iter1++;
        }
      }
    }
    MV_LOG("save virgl cmd total_cmd_size=%dKB valid_cmd_size=%dKB", total_cmd_size >> 10, valid_cmd_size >> 10);

    writer->WriteProtobuf("VGPU", state);
    return VirtioPci::SaveState(writer);
  } 

  bool LoadState(MigrationReader* reader) {
    if (!VirtioPci::LoadState(reader)) {
      return false;
    }

    VirtioVgpuState state;
    if (!reader->ReadProtobuf("VGPU", state)) {
      return false;
    }

    // use staging now to accelerate guest vgpu
    vgpu_config_.staging = true;

    Schedule([this, state]() {
      if (virgl_renderer_init(this, VIRGL_RENDERER_USE_GLX, &vgpu_cbs) != 0) {
        if (virgl_renderer_init(this, VIRGL_RENDERER_USE_EGL | VIRGL_RENDERER_USE_GLES, &vgpu_cbs) != 0) {
          MV_PANIC("init virgl render failed");
        }
      }
      initialized_ = true;

      virgl_renderer_reset();
      reset_ = false;

      size_t total_cmd_size = 0;
      for (int i = 0; i < state.virgl_contexts_size(); i++) {
        auto& virgl_context = state.virgl_contexts(i);

        // create virgl context
        auto ret = virgl_renderer_context_create(virgl_context.context_id(), 0, nullptr);
        if (ret != 0) {
          MV_PANIC("create virgl context=%d failed", virgl_context.context_id());
        }

        // create virgl resources for current virgl context
        for (int j = 0; j < virgl_context.virgl_resources_size(); j++) {
          auto& resource = virgl_context.virgl_resources(j);
          auto args = (struct virgl_renderer_resource_create_args*)resource.resource_args().data();
          
          ret = virgl_renderer_resource_create(args, NULL, 0);
          if (ret != 0) {
            MV_PANIC("create virgl resource=%d failed", args->handle);
          }

          if (resource.size() > 0) {
            auto iov = new iovec {
              .iov_base = manager_->TranslateGuestMemory(resource.gpa()),
              .iov_len = resource.size()
            };

            ret = virgl_renderer_resource_attach_iov(args->handle, iov, 1);
            if (ret != 0) {
              MV_PANIC("attach virgl resource=%d iov=[0x%lx, %ld] failed", args->handle, iov->iov_base, iov->iov_len);
            }
          }

          // bind virgl resource to virgl context
          virgl_renderer_ctx_attach_resource(virgl_context.context_id(), args->handle);
        }

        // replay virgl commands
        for (auto& cmd : virgl_context.virgl_cmds()) {
          auto ret = virgl_renderer_submit_cmd((void*)cmd.data(), virgl_context.context_id(), cmd.size() / 4);
          if (ret) {
            MV_PANIC("submit command err=%d ctx_id=%d size=%d\n", ret, virgl_context.context_id(), cmd.size());
          }
          total_cmd_size += cmd.size();
        }
      }

      MV_LOG("load virgl cmd size=%dKB", total_cmd_size >> 10);
    });
    return true;
  }

  // get information struct from iovec by custom size
  void* EatIovec(std::deque<struct iovec>& iovec, size_t size) {
    MV_ASSERT(!iovec.empty());
    auto& front = iovec.front();
    void* ptr = front.iov_base;

    if (front.iov_len > size) {
      front.iov_len -= size;
      front.iov_base = (uint8_t*)front.iov_base + size;
    } else if (front.iov_len == size) {
      iovec.pop_front();
    } else {
      MV_PANIC("single data struct would never be split into different iovs size=%d front.iov_len=%d back.iov_len=%d iovec_size=%d", 
        size, front.iov_len, iovec.back().iov_len, iovec.size());
    }
    return ptr;
  }

  void GetCapsetInfo(VirtElement* element) {
    auto cmd = (struct virtio_gpu_get_capset_info*)EatIovec(element->vector, sizeof(struct virtio_gpu_get_capset_info));
    auto resp = (struct virtio_gpu_resp_capset_info*)EatIovec(element->vector, sizeof(struct virtio_gpu_resp_capset_info));
    memset(resp, 0, sizeof(*resp));

    switch (cmd->capset_index)
    {
    case 0:
      resp->capset_id = VIRTIO_GPU_CAPSET_VIRGL;
      break;
    case 1:
      resp->capset_id = VIRTIO_GPU_CAPSET_VIRGL2;
      break;
    default:
      MV_PANIC("unknown capset index=%d", cmd->capset_index);
      break;
    }
    virgl_renderer_get_cap_set(resp->capset_id, &resp->capset_max_version, &resp->capset_max_size);

    resp->hdr.type = VIRTIO_GPU_RESP_OK_CAPSET_INFO;
    element->length = sizeof(struct virtio_gpu_resp_capset_info);
  }

  void GetCapset(VirtElement* element) {
    auto cmd = (struct virtio_gpu_get_capset*)EatIovec(element->vector, sizeof(struct virtio_gpu_get_capset));
    auto resp = (struct virtio_gpu_resp_capset*)EatIovec(element->vector, sizeof(struct virtio_gpu_resp_capset));
    
    uint32_t max_ver, max_size;
    virgl_renderer_get_cap_set(cmd->capset_id, &max_ver, &max_size);
    if (!max_size) {
        resp->hdr.type = VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
        return;
    }

    // make sure that buffer of resp is big enough to save caps from virgl
    memset(resp, 0, sizeof(*resp) + max_size);
    virgl_renderer_fill_caps(cmd->capset_id, cmd->capset_version, (void*)resp->capset_data);

    resp->hdr.type = VIRTIO_GPU_RESP_OK_CAPSET;
    element->length = sizeof(*resp) + max_size;
  }

  void CreateVirglContext(VirtElement* element) {
    auto cmd = (struct virtio_gpu_ctx_create*)EatIovec(element->vector, sizeof(struct virtio_gpu_ctx_create));
    auto ret = virgl_renderer_context_create_with_flags(cmd->hdr.ctx_id, cmd->context_init, cmd->nlen, cmd->debug_name);
    if (ret) {
      MV_ERROR("create virgl context failed id=%d debug_name=%s", cmd->hdr.ctx_id, cmd->debug_name);
    }
  }

  void DestroyVirglContext(VirtElement* element) {
    auto cmd = (struct virtio_gpu_ctx_destroy*)EatIovec(element->vector, sizeof(struct virtio_gpu_ctx_destroy));
    virgl_renderer_context_destroy(cmd->hdr.ctx_id);

    // unmap blob regions from this virgl context
    auto regions = blob_memory_regions_[cmd->hdr.ctx_id];
    for (auto iter = regions.begin(); iter != regions.end(); iter++) {
      manager_->machine()->memory_manager()->Unmap(&(iter->second));
      blob_memory_regions_[cmd->hdr.ctx_id].erase(iter);
    }
    blob_memory_regions_.erase(cmd->hdr.ctx_id);

    // guest didn't support migration when staging=true
    if (vgpu_config_.staging) {
      return;
    }
    
    virgl_resources_.erase(cmd->hdr.ctx_id);
  }

  void Create2dResource(VirtElement* element) {
    auto cmd = (struct virtio_gpu_resource_create_2d*)EatIovec(element->vector, sizeof(struct virtio_gpu_resource_create_2d));

    struct virgl_renderer_resource_create_args args = {
      .handle = cmd->resource_id,
      .target = 2,
      .format = cmd->format,
      .bind = 2,
      .width = cmd->width,
      .height = cmd->height,
      .depth = 1,
      .array_size = 1,
      .last_level = 0,
      .nr_samples = 0,
      .flags = VIRTIO_GPU_RESOURCE_FLAG_Y_0_TOP
    };

    auto ret = virgl_renderer_resource_create(&args, NULL, 0);
    if (ret) {
      MV_ERROR("create virgl 2d resource failed id=%d format=%d ret=%d ", cmd->resource_id, cmd->format, ret);
      return;
    }

    // bind virgl resource to virgl context
    virgl_renderer_ctx_attach_resource(cmd->hdr.ctx_id, cmd->resource_id);

    // guest didn't support migration when staging=true
    if (vgpu_config_.staging) {
      return;
    }

    virgl_resources_[cmd->hdr.ctx_id][cmd->resource_id] = {
      .gpa = 0,
      .size = 0,
      .args = std::move(args)
    };
  }

  void Create3dResource(VirtElement* element) {
    auto cmd = (struct virtio_gpu_resource_create_3d*)EatIovec(element->vector, sizeof(struct virtio_gpu_resource_create_3d));
    
    struct virgl_renderer_resource_create_args args = {
      .handle = cmd->resource_id,
      .target = cmd->target,
      .format = cmd->format,
      .bind = cmd->bind,
      .width = cmd->width,
      .height = cmd->height,
      .depth = cmd->depth,
      .array_size = cmd->array_size,
      .last_level = cmd->last_level,
      .nr_samples = cmd->nr_samples,
      .flags = cmd->flags
    };
    
    auto ret = virgl_renderer_resource_create(&args, NULL, 0);
    if (ret) {
      MV_ERROR("create virgl 3d resource failed id=%d ret=%d", cmd->resource_id, ret);
      return;
    }

    // bind virgl resource to virgl context
    virgl_renderer_ctx_attach_resource(cmd->hdr.ctx_id, cmd->resource_id);

    // guest didn't support migration when staging=true
    if (vgpu_config_.staging) {
      return;
    }

    virgl_resources_[cmd->hdr.ctx_id][cmd->resource_id] = {
      .gpa = 0,
      .size = 0,
      .args = std::move(args)
    };
  }

  void CreateBlobResource(VirtElement* element) {
    auto cmd = (struct virtio_gpu_resource_create_blob*)EatIovec(element->vector, sizeof(struct virtio_gpu_resource_create_blob));
    
    // make up pipe resource command to virgl
    uint32_t blob_command[VIRGL_PIPE_RES_CREATE_SIZE + 1] = {0};
    blob_command[0] = VIRGL_CMD0(VIRGL_CCMD_PIPE_RESOURCE_CREATE, 0, VIRGL_PIPE_RES_CREATE_SIZE);
    blob_command[VIRGL_PIPE_RES_CREATE_FORMAT] = cmd->format;
    blob_command[VIRGL_PIPE_RES_CREATE_BIND] = cmd->bind;
    blob_command[VIRGL_PIPE_RES_CREATE_TARGET] = cmd->target;
    blob_command[VIRGL_PIPE_RES_CREATE_WIDTH] = cmd->width;
    blob_command[VIRGL_PIPE_RES_CREATE_HEIGHT] = cmd->height;
    blob_command[VIRGL_PIPE_RES_CREATE_DEPTH] = cmd->depth;
    blob_command[VIRGL_PIPE_RES_CREATE_ARRAY_SIZE] = cmd->array_size;
    blob_command[VIRGL_PIPE_RES_CREATE_LAST_LEVEL] = cmd->last_level;
    blob_command[VIRGL_PIPE_RES_CREATE_NR_SAMPLES] = cmd->nr_samples;
    blob_command[VIRGL_PIPE_RES_CREATE_FLAGS] = cmd->flags;
    blob_command[VIRGL_PIPE_RES_CREATE_BLOB_ID] = cmd->blob_id;

    auto ret = virgl_renderer_submit_cmd(blob_command, cmd->hdr.ctx_id, VIRGL_PIPE_RES_CREATE_SIZE + 1);
    if (ret) {
      MV_ERROR("create pipe resource failed err=%d ctx_id=%d\n", ret, cmd->hdr.ctx_id);
      return;
    }

    struct virgl_renderer_resource_create_blob_args args = {
      .res_handle = cmd->resource_id,
      .ctx_id = cmd->hdr.ctx_id,
      .blob_mem = cmd->blob_mem,
      .blob_flags = cmd->blob_flags,
      .blob_id = cmd->blob_id,
      .size = cmd->size,
      .iovecs = nullptr,
      .num_iovs = 0,
    };

    ret = virgl_renderer_resource_create_blob(&args);
    if (ret) {
      MV_ERROR("create virgl blob resource failed id=%d ret=%d", cmd->resource_id, ret);
      return;
    }
    
    // bind virgl resource to virgl context
    virgl_renderer_ctx_attach_resource(cmd->hdr.ctx_id, cmd->resource_id);
  }

  void MapBlobResource(VirtElement* element) {
    auto cmd = (struct virtio_gpu_resource_map_blob*)EatIovec(element->vector, sizeof(struct virtio_gpu_resource_map_blob));
    auto resp = (struct virtio_gpu_resp_map_info*)EatIovec(element->vector, sizeof(struct virtio_gpu_resp_map_info));
    
    void* map;
    size_t size;
    auto ret = virgl_renderer_resource_map(cmd->resource_id, &map, &size);
    if (ret) {
      MV_ERROR("map virgl blob resource failed id=%d ret=%d", cmd->resource_id, ret);
      return;
    }

    resp->gpa = blob_memory_gpa_base_;
    resp->size = size;
    resp->hdr.type = VIRTIO_GPU_RESP_OK_MAP_INFO;
    virgl_renderer_resource_get_map_info(cmd->resource_id, &resp->map_info);
    
    // map the blob memory from gpu to guest
    auto region = manager_->machine()->memory_manager()->Map(resp->gpa, resp->size, map, kMemoryTypeRam, "VGPU");
    blob_memory_regions_[cmd->hdr.ctx_id][cmd->resource_id] = region;
    blob_memory_gpa_base_ += size;
  }

  void UnmapBlobResource(VirtElement* element) {
    auto cmd = (struct virtio_gpu_resource_unmap_blob*)EatIovec(element->vector, sizeof(struct virtio_gpu_resource_unmap_blob));
    
    auto ret = virgl_renderer_resource_unmap(cmd->resource_id);
    if (ret) {
      MV_ERROR("unmap virgl blob resource failed id=%d ret=%d", cmd->resource_id, ret);
      return;
    }

    auto iter = blob_memory_regions_[cmd->hdr.ctx_id].find(cmd->resource_id);
    if (iter != blob_memory_regions_[cmd->hdr.ctx_id].end()) {
      // unmap the blob memory mapped from gpu
      manager_->machine()->memory_manager()->Unmap(&(iter->second));
      blob_memory_regions_[cmd->hdr.ctx_id].erase(iter);
    }
  }

  void SubmitCommand(VirtElement* element) {
    auto cmd = (struct virtio_gpu_cmd_submit*)EatIovec(element->vector, sizeof(struct virtio_gpu_cmd_submit));

    // we use contiguous physical memory
    MV_ASSERT(element->vector.size() == 1);

    auto& vector = element->vector.front();
    element->vector.pop_front();

    auto ret = virgl_renderer_submit_cmd(vector.iov_base, cmd->hdr.ctx_id, cmd->size / 4);
    if (ret) {
      MV_ERROR("submit command err=%d ctx_id=%d size=%d\n", ret, cmd->hdr.ctx_id, cmd->size / 4);
      return;
    }

    // guest didn't support migration when staging=true
    if (vgpu_config_.staging) {
      return;
    }

    std::string cmd_buf((char*)vector.iov_base, cmd->size);
    virgl_cmds_[cmd->hdr.ctx_id].emplace_back(std::move(cmd_buf));
  }

  void AttachBackingResource(VirtElement* element) {
    auto cmd = (struct virtio_gpu_resource_attach_backing*)EatIovec(element->vector, sizeof(struct virtio_gpu_resource_attach_backing));

    auto iov = new iovec {
      .iov_base = manager_->TranslateGuestMemory(cmd->gpa),
      .iov_len = cmd->size
    };

    auto ret = virgl_renderer_resource_attach_iov(cmd->resource_id, iov, 1);
    if (ret) {
      MV_ERROR("attach backing virgl resource id=%d ret=%d", cmd->resource_id, ret);
      return;
    }

    // guest didn't support migration when staging=true
    if (vgpu_config_.staging) {
      return;
    }

    auto& resource = virgl_resources_[cmd->hdr.ctx_id][cmd->resource_id];
    resource.gpa = cmd->gpa;
    resource.size = cmd->size;
  }

  void DetachBackingResource(VirtElement* element) {
    auto cmd = (struct virtio_gpu_resource_detach_backing*)EatIovec(element->vector, sizeof(struct virtio_gpu_resource_detach_backing));

    int num = 0;
    struct iovec* iov = nullptr;
    virgl_renderer_resource_detach_iov(cmd->resource_id, &iov, &num);
    if (iov != nullptr && num == 1) {
      delete iov;
    } else {
      MV_ERROR("something wrong with resource create/delete iov=0x%lx num=%d", iov, num);
    }
  }

  void AttachResource(VirtElement* element) {
    auto cmd = (struct virtio_gpu_ctx_resource*)EatIovec(element->vector, sizeof(struct virtio_gpu_ctx_resource));
    virgl_renderer_ctx_attach_resource(cmd->hdr.ctx_id, cmd->resource_id);
  }

  void DetachResource(VirtElement* element) {
    auto cmd = (struct virtio_gpu_ctx_resource*)EatIovec(element->vector, sizeof(struct virtio_gpu_ctx_resource));
    virgl_renderer_ctx_detach_resource(cmd->hdr.ctx_id, cmd->resource_id);
  }

  void UnrefResource(VirtElement* element) {
    auto cmd = (struct virtio_gpu_resource_unref*)EatIovec(element->vector, sizeof(struct virtio_gpu_resource_unref));
    virgl_renderer_ctx_detach_resource(cmd->hdr.ctx_id, cmd->resource_id);
    virgl_renderer_resource_unref(cmd->resource_id);

    // guest didn't support migration when staging=true
    if (vgpu_config_.staging) {
      return;
    }
    
    auto iter = virgl_resources_[cmd->hdr.ctx_id].find(cmd->resource_id);
    if (iter != virgl_resources_[cmd->hdr.ctx_id].end()) {
      virgl_resources_[cmd->hdr.ctx_id].erase(iter);
    }
  }

  void TransferHost3d(VirtElement* element, bool to_host) {
    auto cmd = (struct virtio_gpu_transfer_host_3d*)EatIovec(element->vector, sizeof(struct virtio_gpu_transfer_host_3d));
    
    int ret;
    if (to_host) {
      ret = virgl_renderer_transfer_write_iov(cmd->resource_id, cmd->hdr.ctx_id, 
              cmd->level, cmd->stride, cmd->layer_stride, (struct virgl_box *)&cmd->box, cmd->offset, NULL, 0);
    } else {
      ret = virgl_renderer_transfer_read_iov(cmd->resource_id, cmd->hdr.ctx_id, 
              cmd->level, cmd->stride, cmd->layer_stride, (struct virgl_box *)&cmd->box, cmd->offset, NULL, 0);
    }

    if (ret) {
      MV_ERROR("call TransferHost 3d err=%d direct=%d resource_id=%d\n", ret, to_host, cmd->resource_id);
    }
  }

  void TransferHost2d(VirtElement* element) {
    auto cmd = (struct virtio_gpu_transfer_to_host_2d*)EatIovec(element->vector, sizeof(struct virtio_gpu_transfer_to_host_2d));
    
    struct virtio_gpu_box box = {
      .x = cmd->r.x,
      .y = cmd->r.y,
      .z = 0,
      .w = cmd->r.width,
      .h = cmd->r.height,
      .d = 1
    };

    auto ret = virgl_renderer_transfer_write_iov(cmd->resource_id, cmd->hdr.ctx_id, 0, 0, 0, (struct virgl_box *)&box, cmd->offset, NULL, 0);
    if (ret) {
      MV_ERROR("call TransferHost 2d err=%d res_id=%d w=%d h=%d\n", ret, cmd->resource_id, box.w, box.h);
    }
  }

  uint64_t HandleCommand(VirtElement* element) {
    auto cmd_header = (struct virtio_gpu_ctrl_hdr*)element->vector[0].iov_base;
    
    if (!initialized_) {
      if (virgl_renderer_init(this, VIRGL_RENDERER_USE_GLX, &vgpu_cbs) != 0) {
        if (virgl_renderer_init(this, VIRGL_RENDERER_USE_EGL | VIRGL_RENDERER_USE_GLES, &vgpu_cbs) != 0) {
          MV_PANIC("init virgl render failed");
        }
      }
      initialized_ = true;
    }

    if (reset_) {
      virgl_renderer_reset();
      reset_ = false;
    }

    switch (cmd_header->type) {
      case VIRTIO_GPU_CMD_GET_CAPSET_INFO:
        GetCapsetInfo(element);
        break;
      case VIRTIO_GPU_CMD_GET_CAPSET:
        GetCapset(element);
        break;
      case VIRTIO_GPU_CMD_CTX_CREATE:
        CreateVirglContext(element);
        break;
      case VIRTIO_GPU_CMD_CTX_DESTROY:
        DestroyVirglContext(element);
        break;
      case VIRTIO_GPU_CMD_RESOURCE_CREATE_2D:
        Create2dResource(element);
        break;
      case VIRTIO_GPU_CMD_RESOURCE_CREATE_3D:
        Create3dResource(element);
        break;
      case VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING:
        AttachBackingResource(element);
        break;
      case VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING:
        DetachBackingResource(element);
        break;
      case VIRTIO_GPU_CMD_CTX_ATTACH_RESOURCE:
        AttachResource(element);
        break;
      case VIRTIO_GPU_CMD_SUBMIT_3D:
        SubmitCommand(element);
        break;
      case VIRTIO_GPU_CMD_CTX_DETACH_RESOURCE:
        DetachResource(element);
        break;
      case VIRTIO_GPU_CMD_RESOURCE_UNREF:
        UnrefResource(element);
        break;
      case VIRTIO_GPU_CMD_TRANSFER_FROM_HOST_3D:
        TransferHost3d(element, false);
        break;
      case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_3D:
        TransferHost3d(element, true);
        break;
      case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D:
        TransferHost2d(element);
        break;
      case VIRTIO_GPU_CMD_RESOURCE_CREATE_BLOB:
        CreateBlobResource(element);
        break;
      case VIRTIO_GPU_CMD_RESOURCE_MAP_BLOB:
        MapBlobResource(element);
        break;
      case VIRTIO_GPU_CMD_RESOURCE_UNMAP_BLOB:
        UnmapBlobResource(element);
        break;
      default:
        MV_PANIC("unimplement cmd type=%d", cmd_header->type);
        break;
    }

    if (cmd_header->flags & VIRTIO_GPU_FLAG_FENCE) {
      auto ret = virgl_renderer_create_fence(cmd_header->fence_id, cmd_header->ctx_id);
      if (ret != 0) {
        MV_PANIC("virgl_renderer_create_fence failed ret=%d", ret);
      }
      return cmd_header->fence_id;
    }
    return 0;
  }
};

// Why did libvirglrenderer define fence as 32bit, not 64bit?
// libvirglrenderer/ctx0_fence_retire: ctx0 fence_id is created from uint32_t but stored internally as uint64_t,
// so casting back to uint32_t doesn't result in data loss.
static void virgl_write_fence(void *opaque, uint32_t fence) {
  auto vgpu = (VirtioVgpu*)opaque;
  vgpu->WriteFence(fence);
}

static int virgl_get_drm_fd(void *opaque) {
  auto vgpu = (VirtioVgpu*)opaque;
  return vgpu->GetDrmFd();
}

DECLARE_DEVICE(VirtioVgpu);
