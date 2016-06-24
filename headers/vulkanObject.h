#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "vulkan/vulkan.h"
#include "vulkantools.h"
#include "vulkanexamplebase.h"

class VulkanObject
{
protected:
    VkDevice device;
    VulkanExampleBase *exampleBase;
    VkQueue queue;
    VkCommandBuffer moveBurnPoints;
    uint8_t *pBurn;
    uint32_t offset;

    vkTools::UniformData uniformData;

    struct{
        glm::mat4 projection;
        glm::mat4 model;
        glm::mat4 view;
    } ubo;

public:
    virtual void updateModel(glm::mat4 model){
        ubo.model = model;
        updateUniformBuffer();
        //std::cout<<"model " << offset << std::endl;
        memcpy(pBurn+offset, &model, sizeof(model));
        VkSubmitInfo submitInfo = vkTools::initializers::submitInfo();
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &moveBurnPoints;

        // Submit to queue
        vkDeviceWaitIdle(device);
        vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
    }

    void setupBurnCommand(VkQueue mainqueue,VkCommandBuffer cmd){

        queue = mainqueue;
        moveBurnPoints=cmd;
        updateModel(ubo.model);
    }

    void updateProjView(glm::mat4 projection, glm::mat4 view){
        ubo.projection = projection;
        ubo.view = view;
        updateUniformBuffer();
    }
    VulkanObject(VkDevice mdevice, VulkanExampleBase *mexample) : device(mdevice), exampleBase(mexample){

    }

    ~VulkanObject(){
        vkDestroyBuffer(device, uniformData.buffer, nullptr);
        vkFreeMemory(device, uniformData.memory, nullptr);
    }

private:
    void updateUniformBuffer(){
        uint8_t *pData;
        VkResult err = vkMapMemory(device, uniformData.memory, 0, sizeof(ubo), 0, (void **)&pData);
        assert(!err);
        memcpy(pData, &ubo, sizeof(ubo));
        vkUnmapMemory(device, uniformData.memory);
    }
};
