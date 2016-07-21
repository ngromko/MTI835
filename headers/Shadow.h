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

    // Framebuffer for offscreen rendering
    struct FrameBufferAttachment {
        VkImage image;
        VkDeviceMemory mem;
        VkImageView view;
    };
    struct FrameBuffer {
        int32_t width, height;
        VkFramebuffer frameBuffer;
        FrameBufferAttachment color, depth;
        VkRenderPass renderPass;
    } offScreenFrameBuf;

    struct {
        glm::mat4 viewMatrix;
        uint32_t lightIndex;
    }pCostant;

    VkCommandBuffer offScreenCmdBuffer = VK_NULL_HANDLE;
    VkFormat fbDepthFormat;

    void Shadow::prepareCubeMap(VkQueue queue, VkRenderPass renderPass, uint32_t lightCount);
    void prepareOffscreenFramebuffer(VkQueue queue);
    void prepareOffscreenRenderpass();
    void prepareUniformBuffers();
    void updateCubeFace(uint32_t faceIndex, uint32_t light, std::vector<VulkanObject *> objects, VkBuffer *points);
    void preparePipeline(VkPipelineVertexInputStateCreateInfo *verticesState);
    void setupDescriptorSetLayout();
    void setupDescriptorSets();
public:
    ~Shadow();
    vkTools::VulkanTexture *getCubeMapTexture();
    void buildOffscreenCommandBuffer(std::vector<VulkanObject *> objects, VkBuffer *points, uint32_t lightCount);

    Shadow(VkDevice edevice, VkQueue queue, VkRenderPass renderpass, VkFormat depthFormat, VulkanExampleBase *mainClass, uint32_t lightCount, VkPipelineVertexInputStateCreateInfo *verticesState);
    VkCommandBuffer *getCommandBuffer();
};

