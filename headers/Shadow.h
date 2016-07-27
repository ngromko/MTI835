/*
* Vulkan Example - Animated gears using multiple uniform buffers
*
* See readme.md for details
*
* Copyright (C) 2015 by Sascha Willems - www.saschawillems.de
*
* This code is licensed under the MIT license (MIT) (http://opensource.org/licenses/MIT)
*/

#pragma once

#include "vulkan/vulkan.h"
#include "vulkantools.h"
#include "vulkanObject.h"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#define VERTEX_BUFFER_BIND_ID 0

// Texture properties
#define TEX_DIM 1024
#define TEX_FILTER VK_FILTER_LINEAR

// Offscreen frame buffer properties
#define FB_DIM TEX_DIM
#define FB_COLOR_FORMAT VK_FORMAT_R32_SFLOAT

// Framebuffer for offscreen rendering
struct FrameBufferAttachment {
    VkImage image;
    VkDeviceMemory mem;
    VkImageView view;
    VkFormat format;
};
struct FrameBuffer {
    int32_t width, height;
    VkFramebuffer frameBuffer;
    std::vector<FrameBufferAttachment> attachments;
    VkRenderPass renderPass;
    VkSampler sampler;
    void FreeResources(VkDevice device)
    {
        for (auto attachment : attachments)
        {
            vkDestroyImage(device, attachment.image, nullptr);
            vkDestroyImageView(device, attachment.view, nullptr);
            vkFreeMemory(device, attachment.mem, nullptr);
        }
        vkDestroySampler(device, sampler, nullptr);
        vkDestroyRenderPass(device, renderPass, nullptr);
        vkDestroyFramebuffer(device, frameBuffer, nullptr);
    }
};

class Shadow
{
private:
    vkTools::UniformData offscreen;
    struct {
        glm::mat4 projection;
        glm::vec3 lightPos[2];
    } uboOffscreenVS;

    VulkanExampleBase *exampleBase;
    VkDevice device;

    float zNear = 0.1f;
    float zFar = 1024.0f;

    VkPipeline shadowPipeline;
    VkPipelineLayout shadowPipelineLayout;

    VkRenderPassBeginInfo renderPassBeginInfo;

    VkDescriptorSet descSet;

    VkDescriptorSetLayout descriptorSetLayout;

    vkTools::VulkanTexture shadowCubeMap;

    FrameBuffer offScreenFrameBuf;

    struct PConst {
        glm::mat4 viewMatrix;
        glm::u32vec4 lightIndex;
    }pCostant;

    //VkCommandBuffer offScreenCmdBuffer = VK_NULL_HANDLE;

    void Shadow::prepareCubeMap(VkQueue queue);
    void prepareOffscreenFramebuffer(VkQueue queue, VkFormat fbDepthFormat);
    void prepareOffscreenRenderpass(VkFormat fbDepthFormat);
    void prepareUniformBuffers();
    void updateCubeFace(uint32_t faceIndex, uint32_t light, std::vector<VulkanObject *> objects, VkBuffer *points);
    void preparePipeline(VkPipelineVertexInputStateCreateInfo *verticesState);
    void setupDescriptorSetLayout();
    void updateCubeFace(VkCommandBuffer offScreenCmdBuffer, uint32_t faceIndex, uint32_t light, std::vector<VulkanObject *> objects, VkBuffer *points);
    void setupDescriptorSets(VkDescriptorBufferInfo *lights);
public:
    ~Shadow();
    vkTools::VulkanTexture *getCubeMapTexture();
    void buildOffscreenCommandBuffer(std::vector<VulkanObject *> objects, VkBuffer *points, uint32_t lightCount);

    VkCommandBuffer *getCommandBuffer();
    void buildOffscreenCommandBuffer(VkCommandBuffer offScreenCmdBuffer, std::vector<VulkanObject *> objects, VkBuffer *points, uint32_t lightIndex);
    Shadow(VkDevice edevice, VkQueue queue, VkDescriptorBufferInfo *lights, VkFormat depthFormat, VulkanExampleBase *mainClass, VkPipelineVertexInputStateCreateInfo *verticesState);
};

