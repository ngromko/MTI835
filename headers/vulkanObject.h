#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>

#include "vulkan/vulkan.h"
#include "vulkantools.h"



class VulkanObject
{
protected:
    VkDevice device;
    vkTools::UniformData uniformData;
    struct{
        glm::mat4 projection;
        glm::mat4 model;
        glm::mat4 view;
    } ubo;

public:
    void updateModel(glm::mat4 model){
        ubo.model = model;
        updateUniformBuffer();
    }

    void updateProjView(glm::mat4 projection, glm::mat4 view){
        ubo.projection = projection;
        ubo.view = view;
        updateUniformBuffer();
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
